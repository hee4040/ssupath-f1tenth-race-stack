#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "f110_msgs/msg/obstacle.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "mpcc_ros/msg/s_hint.hpp"
#include "mpcc_ros/msg/corridor.hpp"

#include "corridor_generator.hpp"
#include "NodeGraph.hpp"
#include "offline_params.hpp"
#include "frenet_conversion_cpp/frenet_converter_cpp.hpp"
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <iostream>

double get_wall_time() {
    struct timeval time;
    if (gettimeofday(&time, NULL)) {
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}


double get_cpu_time() {
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    double user = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec * 1e-6;
    double sys  = usage.ru_stime.tv_sec + usage.ru_stime.tv_usec * 1e-6;
    return user + sys;
}

class VpForwardBackward {
public:
    VpForwardBackward(double ay_max=8.0, double ax_max=5.0, double vel_max=15.0, double gg_scale=1.0)
    : ay_max_(ay_max), ax_max_(ax_max), vel_max_(vel_max), gg_scale_(gg_scale) {}

    void updateDynParameters(double vel_max, double gg_scale) {
        vel_max_ = vel_max;
        gg_scale_ = gg_scale;
    }

    // kappa: size N, el_lengths: size N-1
    Eigen::VectorXd calcVelProfile(const Eigen::VectorXd& kappa,
                                   const Eigen::VectorXd& el_lengths,
                                   double v_start,
                                   double v_end)
    {
        int N = (int)kappa.size();
        Eigen::VectorXd vx(N);

        // 곡률 기반 속도 제한
        Eigen::VectorXd v_lat_max(N);
        for (int i = 0; i < N; ++i) {
            double ki = std::abs(kappa(i));
            v_lat_max(i) = (ki < 1e-6) ? vel_max_ : std::min(std::sqrt(ay_max_*gg_scale_/ki), vel_max_);
        }

        // Forward pass (가속)
        vx(0) = std::min(v_start, v_lat_max(0));
        for (int i = 1;i < N; ++i) {
            double ds = std::max(1e-6, el_lengths(i-1));
            double v_possible = std::sqrt(std::max(0.0, vx(i-1)*vx(i-1) + 2.0*ax_max_*gg_scale_*ds));
            vx(i) = std::min(v_possible, v_lat_max(i));
        }

        // Backward pass (감속)
        vx(N-1) = std::min(vx(N-1), v_end);
        for (int i = N-2; i >= 0; --i) {
            double ds = std::max(1e-6, el_lengths(i));
            double v_possible = std::sqrt(std::max(0.0, vx(i+1)*vx(i+1) + 2.0*ax_max_*gg_scale_*ds));
            vx(i) = std::min(vx(i), v_possible);
        }
        return vx;
    }

private:
    double ay_max_, ax_max_, vel_max_, gg_scale_;
};

// 가속도 추정: a ≈ (v_{i+1}^2 - v_i^2)/(2*ds)
inline Eigen::VectorXd accelFromProfile(const Eigen::VectorXd& vx,
                                        const Eigen::VectorXd& el_lengths)
{
    int N = (int)vx.size();
    Eigen::VectorXd ax(N); ax.setZero();
    for (int i=0;i<N-1;++i) {
        double ds = std::max(1e-6, el_lengths(i));
        ax(i) = (vx(i+1)*vx(i+1) - vx(i)*vx(i)) / (2.0*ds);
    }
    ax(N-1) = ax(N-2);
    return ax;
}

class CorridorGenerator : public rclcpp::Node {
public:
    CorridorGenerator(): Node("corridor_generator"), planning_done(false) {
        // offline parameter
        params = load_offline_params(this);
        // online parameter
        this->declare_parameter<double>("obs_delay_d", 0.0);
        this->declare_parameter<double>("obs_delay_s", 0.0);
        this->declare_parameter<int>("inflate_idx", 0);  
        this->declare_parameter<int>("min_plan_horizon", 1);
        this->declare_parameter<double>("obs_traj_tresh", 2.0);
        this->declare_parameter<double>("closest_obs", 2.0);
        this->declare_parameter<double>("obs_lookahead", 4.0);
        this->declare_parameter<double>("hyst_time", 0.0);

        obs_delay_s_    = this->get_parameter("obs_delay_s").as_double();
        obs_delay_d_        = this->get_parameter("obs_delay_d").as_double();
        min_plan_horizon_   = this->get_parameter("min_plan_horizon").as_int();
        inflate_idx_        = this->get_parameter("inflate_idx").as_int();
        obs_traj_tresh_     = this->get_parameter("obs_traj_tresh").as_double();
        closest_obs_        = this->get_parameter("closest_obs").as_double();
        obs_lookahead_      = this->get_parameter("obs_lookahead").as_double();
        hyst_time_          = this->get_parameter("hyst_time").as_double();
        
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CorridorGenerator::paramCB, this, std::placeholders::_1));

        this->declare_parameter<bool>("from_bag", false);
        this->declare_parameter<bool>("measure", false);
        from_bag = this->get_parameter("from_bag").as_bool();
        measuring = this->get_parameter("measure").as_bool();
        
        rclcpp::QoS qos(1);
        qos.transient_local().reliable();

        // Subscriber - Online
        obs_sub = this->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/perception/obstacles", 1, std::bind(&CorridorGenerator::obs_cb, this, std::placeholders::_1));
        // obs_sub = this->create_subscription<f110_msgs::msg::ObstacleArray>(
        //     "/static_obs", 1, std::bind(&CorridorGenerator::obs_cb, this, std::placeholders::_1));
    
        s_hint_sub = this->create_subscription<mpcc_ros::msg::SHint>(
            "/mpcc/s_hint", 1, std::bind(&CorridorGenerator::s_hint_cb, this, std::placeholders::_1));

        // publisher 
        // 생성자 안에서 초기화
        wpnts_mrks_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/avoidance/markers", qos);
        evasion_pub = this->create_publisher<f110_msgs::msg::OTWpntArray>("/planner/avoidance/otwpnts", qos);
        
        corridor_mrks_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/corridor_markers", qos);
        pub_propagated = this->create_publisher<visualization_msgs::msg::Marker>("/planner/avoidance/propagated_obs", qos);
        corridor_pub = this->create_publisher<mpcc_ros::msg::Corridor>("/mpcc/corridor", qos);

        if (measuring) {
            latency_pub = this->create_publisher<std_msgs::msg::Float32>("/corridor_generator/avoidance/latency", qos);
        }

        // Wait for initial messages (blocking-like behavior but safe)
        RCLCPP_INFO(this->get_logger(), "Waiting for initial messages...");

        loadLtplWaypoints("teras");

        RCLCPP_INFO(this->get_logger(), "All required messages received. Continuing...");

        // Offline Part run -> once
        if (!planning_done) runOffline();

        // create timer at 20 Hz
        timer_ = this->create_wall_timer(25ms, std::bind(&CorridorGenerator::online_loop, this));
    }
    
private:
    // Subscriber
    // rclcpp::Subscription<f110_msgs::msg::LtplWpntArray>::SharedPtr ltpl_waypoints_sub;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;

    rclcpp::Subscription<mpcc_ros::msg::SHint>::SharedPtr s_hint_sub;
    rclcpp::Publisher<mpcc_ros::msg::Corridor>::SharedPtr corridor_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_mrks_pub;

    rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr evasion_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wpnts_mrks_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_propagated;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub;

    // OfflineParams params;
    OfflineParams params;  
    bool measuring{false};
    bool from_bag{false};
    bool planning_done;

    // last_switch_time/side
    rclcpp::Time last_switch_time{0,0,RCL_ROS_TIME};    
    string last_side;   

    // state variables
    double cur_s{0.0};
    double wpnt_max_s{0.0};
    int inflate_idx_{2};
    int min_plan_horizon_;
    double obs_traj_tresh_;
    double closest_obs_;
    double obs_lookahead_;
    double obs_delay_d_;
    double obs_delay_s_;
    double hyst_time_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    DMap gtMap;
    DMap stMap;
    NodeMap nodeMap;
    IVector nodeIndicesOnRaceline;
    f110_msgs::msg::ObstacleArray obs_msg;
    mpcc_ros::msg::Corridor last_corridor;
    rclcpp::TimerBase::SharedPtr timer_;
    bool have_state{false}, have_ltpl{false};
    NodeGraph nodeGraph;
    std::string final_csv_path;
    std::unique_ptr<FrenetConverter> converter;
    double gb_vmax;

    DVector s_hint;
    double cur_x, cur_y;

    // ---- 동적 파라미터 콜백 ----
    rcl_interfaces::msg::SetParametersResult paramCB(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &p : params) {
            if (p.get_name() == "inflate_idx") {
                inflate_idx_ = p.as_int();
                RCLCPP_INFO(this->get_logger(), "inflate_idx updated: %d", inflate_idx_);
            } else if (p.get_name() == "obs_traj_tresh") {
                obs_traj_tresh_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_traj_tresh updated: %.3f", obs_traj_tresh_);
            } else if (p.get_name() == "closest_obs") {
                closest_obs_ = p.as_double();
                RCLCPP_INFO(get_logger(), "closest_obs updated: %.3f", closest_obs_);
            } else if (p.get_name() == "obs_lookahead") {
                obs_lookahead_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_lookahead updated: %.3f", obs_lookahead_);
            } else if (p.get_name() == "obs_delay_d") {
                obs_delay_d_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_delay_d updated: %.3f", obs_delay_d_);
            } else if (p.get_name() == "obs_delay_s") {
                obs_delay_s_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_delay_s updated: %.3f", obs_delay_s_);
            } else if (p.get_name() == "min_plan_horizon") {
                min_plan_horizon_ = p.as_int();
                RCLCPP_INFO(get_logger(), "min_plan_horizon updated: %d", min_plan_horizon_);
            } else if (p.get_name() == "hyst_time") {
                hyst_time_ = p.as_double();
                RCLCPP_INFO(get_logger(), "hyst_time updated: %.3f", hyst_time_);
            }
        }
        return result;
    }

    void s_hint_cb(const mpcc_ros::msg::SHint::SharedPtr msg) {
        s_hint = msg->s;
        cur_x = msg->x;
        cur_y = msg->y;
        cur_s = s_hint.front();

        // 로깅
        // if (!s_hint.empty()) {
        //     RCLCPP_INFO(this->get_logger(),
        //         "[s_hint_cb] Received SHint: size=%zu | cur_s=%.3f | cur_x=%.3f | cur_y=%.3f",
        //         s_hint.size(), cur_s, cur_x, cur_y);
        // } else {
        //     RCLCPP_WARN(this->get_logger(),
        //         "[s_hint_cb] Received SHint with empty s array! cur_x=%.3f | cur_y=%.3f",
        //         cur_x, cur_y);
        // }
    }

    // Callback
    void loadLtplWaypoints(const std::string &map_name) {
        std::string json_file = "/home/misys/forza_ws/race_stack/stack_master/maps/" 
                                + map_name + "/ltpl_waypoints.json";
        std::ifstream f(json_file);
        if (!f.is_open()) {
            throw std::runtime_error("Failed to open ltpl_waypoints.json: " + json_file);
        }

        json j;
        f >> j;

        const auto& wpnts = j["ltpl_traj_wpnts"]["ltplwpnts"];

        gtMap.clear();

        for (const auto &wp : wpnts) {
            gtMap["x_ref_m"].push_back(wp["x_ref_m"].get<double>());
            gtMap["y_ref_m"].push_back(wp["y_ref_m"].get<double>());
            gtMap["width_right_m"].push_back(wp["width_right_m"].get<double>());
            gtMap["width_left_m"].push_back(wp["width_left_m"].get<double>());
            gtMap["x_normvec_m"].push_back(wp["x_normvec_m"].get<double>());
            gtMap["y_normvec_m"].push_back(wp["y_normvec_m"].get<double>());
            gtMap["alpha_m"].push_back(wp["alpha_m"].get<double>());
            gtMap["s_racetraj_m"].push_back(wp["s_racetraj_m"].get<double>());
            gtMap["psi_racetraj_rad"].push_back(wp["psi_racetraj_rad"].get<double>());
            gtMap["kappa_racetraj_radpm"].push_back(wp["kappa_racetraj_radpm"].get<double>());
            gtMap["vx_racetraj_mps"].push_back(wp["vx_racetraj_mps"].get<double>());
            gtMap["ax_racetraj_mps2"].push_back(wp["ax_racetraj_mps2"].get<double>());
        }

        wpnt_max_s = gtMap["s_racetraj_m"].back();

        auto vmax_it = std::max_element(gtMap["vx_racetraj_mps"].begin(),
                                        gtMap["vx_racetraj_mps"].end());
        double gb_vmax = (vmax_it != gtMap["vx_racetraj_mps"].end()) ? *vmax_it : 0.0;

        have_ltpl = true;

        RCLCPP_INFO(this->get_logger(), 
                    "Loaded %zu waypoints from %s (vmax=%.2f, max_s=%.2f)",
                    gtMap["x_ref_m"].size(), json_file.c_str(), gb_vmax, wpnt_max_s);
    }


    void obs_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
        obs_msg = *msg;
    }

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////// Offline Part ////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    void runOffline() {
        try {
            // map_size(gtMap); 
            loadGlobalTrajectoryMap();

            stMap = createSampledTrajectoryMap(gtMap);

            auto [nodeMap, nodeIndicesOnRaceline] = createNodeMap(stMap);

            nodeGraph.setParams(params);
            nodeGraph.setNumLayers(nodeMap);
            nodeGraph.genEdges(nodeMap, nodeIndicesOnRaceline, this->get_logger());
            // nodeGraph.pruneEdges(nodeMap, stMap[RL_VX]);
            // nodeGraph.computeSplineCost(nodeMap, nodeIndicesOnRaceline);
            nodeGraph.computeSplineCost(nodeIndicesOnRaceline);
            // if (!nodeGraph.writeSplineMapToCSV(paramsfinal_csv_path)) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to write CSV to path: %s", params.csv_output_path);
            //     throw std::runtime_error("CSV write failed");
            // }
            
            nodeGraph.printGraph(this->get_logger());
            final_csv_path = params.csv_output_path + params.map_name + "/SplineMap.csv";
            nodeGraph.writeSplineMapToCSV(final_csv_path, this->get_logger());

            RCLCPP_INFO(this->get_logger(), "Offline planning completed successfully!");    
            planning_done = true;

            // Visualize Offline Result 
            // visualizeTrajectories(gtMap, stMap, nodeMap, nodeGraph.getSplineMap());
        }   
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in runOffline: %s", e.what());
            throw;
        }
    }
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Online Part /////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    auto runOnline()
    -> std::tuple<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray, mpcc_ros::msg::Corridor, visualization_msgs::msg::MarkerArray>
    {
        mpcc_ros::msg::Corridor corridor_array;
        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray corridor_mrks;
        visualization_msgs::msg::MarkerArray wpnts_mrks;

        // ===== 공용 람다: s_q 위치에서 보간하여 좌표/법선 반환 =====
        auto interpLayer = [&](double s_q) {
            const auto &S = stMap[RL_S];
            if (S.empty()) return std::tuple<double,double,double,double>(0,0,1,0);

            auto it = std::lower_bound(S.begin(), S.end(), s_q);
            if (it == S.begin()) {
                int idx = 0;
                return std::tuple<double,double,double,double>(
                    stMap[POS_X][idx], stMap[POS_Y][idx],
                    stMap[NORM_X][idx], stMap[NORM_Y][idx]
                );
            }
            if (it == S.end()) {
                int idx = (int)S.size()-1;
                return std::tuple<double,double,double,double>(
                    stMap[POS_X][idx], stMap[POS_Y][idx],
                    stMap[NORM_X][idx], stMap[NORM_Y][idx]
                );
            }

            int idx_next = int(it - S.begin());
            int idx_prev = idx_next - 1;

            double s0 = S[idx_prev], s1 = S[idx_next];
            double t = (s_q - s0) / std::max(1e-9, (s1 - s0));

            double cx = (1-t) * stMap[POS_X][idx_prev] + t * stMap[POS_X][idx_next];
            double cy = (1-t) * stMap[POS_Y][idx_prev] + t * stMap[POS_Y][idx_next];
            double nx = (1-t) * stMap[NORM_X][idx_prev] + t * stMap[NORM_X][idx_next];
            double ny = (1-t) * stMap[NORM_Y][idx_prev] + t * stMap[NORM_Y][idx_next];

            return std::tuple<double,double,double,double>(cx, cy, nx, ny);
        };


        // ===== 공용 람다: s_q로 레이싱 인덱스/노드수를 "앞뒤 보간" =====
        auto interpSHint = [&](double s_q, double &raceline_index_d, double &num_nodes_d) {
            const auto &S = stMap[RL_S];
            auto it = std::lower_bound(S.begin(), S.end(), s_q);
            if (it == S.begin()) {
                int layer = 0;
                raceline_index_d = (double)nodeIndicesOnRaceline[layer];
                num_nodes_d      = (double)nodeMap[layer].size();
                return;
            }
            if (it == S.end()) {
                int layer = (int)S.size() - 1;
                raceline_index_d = (double)nodeIndicesOnRaceline[layer];
                num_nodes_d      = (double)nodeMap[layer].size();
                return;
            }
            int idx_next = int(it - S.begin());
            int idx_prev = idx_next - 1;
            double s0 = S[idx_prev], s1 = S[idx_next];
            double t  = (s_q - s0) / std::max(1e-9, (s1 - s0)); // [0,1] 보장
            t = std::clamp(t, 0.0, 1.0);

            double rl0 = (double)nodeIndicesOnRaceline[idx_prev];
            double rl1 = (double)nodeIndicesOnRaceline[idx_next];
            raceline_index_d = rl0 + t * (rl1 - rl0);

            double n0 = (double)nodeMap[idx_prev].size();
            double n1 = (double)nodeMap[idx_next].size();
            num_nodes_d = n0 + t * (n1 - n0);
        };

        // ===== 공용 람다: s_hint 기반 마커를 생성 (d_left/right_interp 사용) =====
        auto genCorridorMarkers = [&](const DVector &d_left_interp,
                                            const DVector &d_right_interp) {
            visualization_msgs::msg::Marker marker_left, marker_right;

            // 공통 속성
            marker_left.header.frame_id = "map";
            marker_left.header.stamp    = this->now();
            marker_left.ns              = "corridor_left";
            marker_left.id              = 0;
            marker_left.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            marker_left.action          = visualization_msgs::msg::Marker::ADD;
            marker_left.scale.x         = 0.05;
            marker_left.color.r         = 1.0;
            marker_left.color.g         = 0.0;
            marker_left.color.b         = 0.0;
            marker_left.color.a         = 1.0;

            marker_right = marker_left;
            marker_right.ns      = "corridor_right";
            marker_right.id      = 1;
            marker_right.color.r = 0.0;
            marker_right.color.g = 1.0;
            marker_right.color.b = 0.0;

            // s_hint 그리드로만 마커 포인트 생성
            for (size_t i = 0; i < s_hint.size(); ++i) {
                auto [cx, cy, nx, ny] = interpLayer(s_hint[i]);

                geometry_msgs::msg::Point pL, pR;
                pL.x = cx - d_left_interp[i]  * nx;
                pL.y = cy - d_left_interp[i]  * ny;
                pL.z = 0.0;

                pR.x = cx + d_right_interp[i] * nx;
                pR.y = cy + d_right_interp[i] * ny;
                pR.z = 0.0;

                marker_left.points.push_back(pL);
                marker_right.points.push_back(pR);
            }

            corridor_mrks.markers.push_back(marker_left);
            corridor_mrks.markers.push_back(marker_right);
        };

        // ================ 1) 회피 대상 obstacle 필터링 ================
        auto obs = obs_filtering();

        // ================ 2) 장애물 없을 때: s_hint 해상도로 출력 ================
        if (obs.empty()) {
            // RCLCPP_INFO(this->get_logger(), "No obstacles → publishing full corridor (s_hint-aligned).");

            DVector d_left_interp, d_right_interp;
            d_left_interp.reserve(s_hint.size());
            d_right_interp.reserve(s_hint.size());

            for (size_t i = 0; i < s_hint.size(); ++i) {
                if (s_hint[i] >= stMap[RL_S].back()) {
                    s_hint[i] -= stMap[RL_S].back();
                }
            }

            for (double s_q : s_hint) {
                // s_q에서 raceline index / node 수를 앞뒤 레이어 기준 보간하여 추정
                double rl_idx_d = 0.0, num_nodes_d = 1.0;
                interpSHint(s_q, rl_idx_d, num_nodes_d);

                // 0 ~ (num_nodes-1) 대비 raceline offset
                double d_l = rl_idx_d * params.lat_resolution;
                double d_r = (num_nodes_d - 1.0 - rl_idx_d)* params.lat_resolution;

                d_left_interp.push_back(d_l);
                d_right_interp.push_back(d_r);
            }

            // publish (s_hint )
            corridor_array.d_left  = d_left_interp;
            corridor_array.d_right = d_right_interp;
            corridor_array.init_s  = s_hint.front(); // Lap s_hint 기준으로 시작 s

            //  마커도 s_hint 기준 생성
            genCorridorMarkers(d_left_interp, d_right_interp);

            return {wpnts, wpnts_mrks, corridor_array, corridor_mrks};
        }

        RCLCPP_INFO(this->get_logger(), "Start to generate Evasion Corridor");
        // ================ 3) 장애물 있을 때: 블록 구역 계산, 그래프 탐색 ================
        vector<tuple<int,int,int>> blocked_zones;
        bool avoidance_possible = true;

        for (auto &target_obs : obs) {
            auto blocked_ranges = find_obs_zone(target_obs); 

            // 회피 불가 시 뒤 계산하지 않도록 함. (+) 이 경우 state == trailing이어야 함.
            // for (auto &[layer, idx_min, idx_max] : blocked_ranges) {
            //     int total_nodes = (int)nodeMap[layer].size();
            //     int blocked     = idx_max - idx_min + 1;

            //     if ((total_nodes - blocked) * params.lat_resolution >= params.veh_width ) {
            //         RCLCPP_INFO(this->get_logger(),
            //                     "Layer %d blocked too much (%d/%d). Avoidance impossible.",
            //                     layer, blocked, total_nodes);
            //         avoidance_possible = false;
            //         break;
            //     }
            // }

            // if (!avoidance_possible) break;

            blocked_zones.insert(blocked_zones.end(),
                                blocked_ranges.begin(),
                                blocked_ranges.end());
        }

        if (!avoidance_possible) {
            //  회피 불가 시 빈 백터 반환
            RCLCPP_INFO(this->get_logger(), "Skipping path search, fallback (empty corridor).");
            return {wpnts, wpnts_mrks, corridor_array, corridor_mrks};
        }

        //  노드/엣지 가중치 상승
        nodeGraph.apply_node_filter(blocked_zones);

        //  시작/목표 인덱스
        auto [startIdx, endIdx] = findDestination();

        //  최단 경로 탐색
        IPairVector nodeArray = nodeGraph.graph_search(startIdx, endIdx, nodeIndicesOnRaceline, this->get_logger());

        //  필터 해제
        nodeGraph.deactivateFiltering();

        if (nodeArray.size() < 2) return {wpnts, wpnts_mrks, corridor_array, corridor_mrks};

        //  마지막 노드가 레이싱라인이 아니면 강제 보정 (쿠션용)
        auto [check_layer, check_idx] = nodeArray.back();
        if (nodeIndicesOnRaceline[check_layer] != check_idx) {
            RCLCPP_INFO(this->get_logger(), "Last Node isn't on Raceline → snapping to raceline.");
            int rl_idx = nodeIndicesOnRaceline[check_layer];
            nodeArray.back() = {check_layer, rl_idx};
        }
        
        //  nodeArray → path_layers(레이어 중복 제거)
        IVector path_layers;
        path_layers.reserve(nodeArray.size());
        int last_layer = -1;
        for (auto &p : nodeArray) {
            int L = p.first;
            if (L != last_layer) {
                path_layers.push_back(L);
                last_layer = L;
            }
        }

        if (path_layers.empty()) return {wpnts, wpnts_mrks, corridor_array, corridor_mrks};

        // RCLCPP_INFO(this->get_logger(),
        //             "[Corridor] Published corridor with %zu points (aligned to s_hint)",
        //             s_hint.size());

        ////////////////////////////////////////////////////////////////////        
        /////////////////////////// corridor 저장 ///////////////////////////
        ////////////////////////////////////////////////////////////////////
        auto [d_left_interp, d_right_interp] = genCorridor(startIdx, endIdx, blocked_zones, path_layers);
        // corridor_array 저장
        corridor_array.d_left  = d_left_interp;
        corridor_array.d_right = d_right_interp;
        corridor_array.init_s  = s_hint.front(); //  s_hint 기준 시작 s
        // corridor markers 저장
        genCorridorMarkers(d_left_interp, d_right_interp);
        
        ////////////////////////////////////////////////////////////////////        
        //////////////////////////// 회피경로 저장 ////////////////////////////
        ////////////////////////////////////////////////////////////////////
        // if (!nodeArray.empty() && nodeArray.front() != startIdx) {
        //     std::reverse(nodeArray.begin(), nodeArray.end());
        // }
        int evasion_horizon = 20;  
        if ((int)nodeArray.size() > evasion_horizon) {
            nodeArray.resize(evasion_horizon);
        }

        auto [evasion_wpnts, evasion_wpnts_mrks] = genWpnts(nodeArray);

        wpnts = evasion_wpnts;
        wpnts_mrks = evasion_wpnts_mrks;
        

        return {wpnts, wpnts_mrks, corridor_array, corridor_mrks};
    }

    auto genCorridor(IPair& startIdx, IPair& endIdx, vector<tuple<int,int,int>>& blocked_zones, IVector& path_layers) -> pair<DVector, DVector> {
        std::unordered_map<int, IPair> corridor_limits;

        for (int layer = startIdx.first; layer <= endIdx.first; ++layer) {
            int raceline_index = nodeIndicesOnRaceline[layer];
            int min_idx = 0;
            int max_idx = (int)nodeMap[layer].size() - 1;

            for (auto &[blk_layer, idx_min, idx_max] : blocked_zones) {
                if (blk_layer != layer) continue;

                if (idx_min <= raceline_index && raceline_index <= idx_max) {
                    int left_free  = raceline_index - min_idx;
                    int right_free = max_idx - raceline_index;
                    if (left_free > right_free) {
                        max_idx = idx_min - 1;
                    } else {
                        min_idx = idx_max + 1;
                    }
                } else {
                    if (idx_min <= max_idx && idx_max >= min_idx) {
                        if (idx_min <= raceline_index) {
                            min_idx = std::max(min_idx, idx_max + 1);
                        }
                        if (idx_max >= raceline_index) {
                            max_idx = std::min(max_idx, idx_min - 1);
                        }
                    }
                }
            }

            if (min_idx > max_idx) {
                RCLCPP_WARN(this->get_logger(), "Layer %d fully blocked. Corridor collapsed!", layer);
                continue;
            }

            corridor_limits[layer] = {min_idx, max_idx};
            RCLCPP_INFO(this->get_logger(),
                        "Layer %d corridor range: min=%d, raceline=%d, max=%d",
                        layer, min_idx, raceline_index, max_idx);
        }

        // ================ 5) path_layers만 순회해 d_left/d_right/s_array 산출 ================
        DVector d_left, d_right, s_array;
        d_left.reserve(path_layers.size());
        d_right.reserve(path_layers.size());
        s_array.reserve(path_layers.size());

        for (int layer : path_layers) {
            auto it = corridor_limits.find(layer);
            if (it == corridor_limits.end()) continue;

            auto [min_idx, max_idx] = it->second;
            int raceline_index = nodeIndicesOnRaceline[layer];
            int num_nodes = (int)nodeMap[layer].size();
            if (num_nodes <= 0 || raceline_index < 0 || raceline_index >= num_nodes) continue;

            //  raceline 기준 offset (왼쪽: raceline - min, 오른쪽: max - raceline)
            double d_l = (raceline_index - min_idx) * params.lat_resolution;
            double d_r = (max_idx - raceline_index) * params.lat_resolution;

            d_left.push_back(d_l);
            d_right.push_back(d_r);
            s_array.push_back(stMap[RL_S][layer]);
        }

        // ================ 6) s_hint에 맞게 선형 보간 ================
        DVector d_left_interp, d_right_interp;
        d_left_interp.reserve(s_hint.size());
        d_right_interp.reserve(s_hint.size());

        for (size_t i = 0; i < s_hint.size(); ++i) {
            if (s_hint[i] >= stMap[RL_S].back()) {
                s_hint[i] -= stMap[RL_S].back();
            }
        }

        for (double s_q : s_hint) {
            auto it = std::lower_bound(s_array.begin(), s_array.end(), s_q);

            if (it == s_array.begin()) {
                d_left_interp.push_back(d_left.front());
                d_right_interp.push_back(d_right.front());
                continue;
            }
            if (it == s_array.end()) {
                d_left_interp.push_back(d_left.back());
                d_right_interp.push_back(d_right.back());
                continue;
            }

            int idx_next = int(it - s_array.begin());
            int idx_prev = idx_next - 1;

            double s0 = s_array[idx_prev], s1 = s_array[idx_next];
            double t  = (s_q - s0) / std::max(1e-9, (s1 - s0));
            t = std::clamp(t, 0.0, 1.0);

            double dl = d_left[idx_prev]  + t * (d_left[idx_next]  - d_left[idx_prev]);
            double dr = d_right[idx_prev] + t * (d_right[idx_next] - d_right[idx_prev]);

            d_left_interp.push_back(dl);
            d_right_interp.push_back(dr);

        }
        return {d_left_interp, d_right_interp};
    
    }      

    auto genWpnts(IPairVector nodeArray) ->  pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray> {
        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray wpnts_mrks;
        // 현재 spline 상태 x 노드 시퀀스이므로 spline 게산을 위한 작업 실행
        MatrixXd path(nodeArray.size(), 2);
        // 경로를 Eigen::Matrix 타입으로 변경 
        // 기존 node 구조체에 들어있는 x, y, psi 재사용
        for (size_t i = 0; i < nodeArray.size(); ++i) {
            auto [layer, idx] = nodeArray[i];
            const ::Node &n = nodeMap[layer][idx];  
            path(i, 0) = n.x;
            path(i, 1) = n.y;
        }
        double psi_s = nodeMap[nodeArray.front().first][nodeArray.front().second].psi;
        double psi_e = nodeMap[nodeArray.back().first][nodeArray.back().second].psi;

        // 최소 비용 경로의 뼈대인 노드 시퀀스를 구간별 spline으로 잇는 작업
        auto evasion_spline = nodeGraph.computeSplines(path, psi_s, psi_e, true);

        if (evasion_spline->coeffs_x.size() == 0 || evasion_spline->coeffs_y.size() == 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Skipping waypoint generation: empty spline coefficients (path too short)");
            return {wpnts, wpnts_mrks};
        }

        // 토픽으로 내보내기 위해 spline 위 점 sampling
        auto evasion_points = nodeGraph.interpSpline(evasion_spline->coeffs_x,
                                                evasion_spline->coeffs_y);
        
        // 곡률/구간거리(el_lengths) 계산
        int N = (int)evasion_points.size();
        Eigen::VectorXd kappa(N);
        Eigen::VectorXd el_lengths(N-1);
        
        kappa(0) = evasion_points[0].kappa;
        for (int i=1;i<N;++i) {
            kappa(i) = evasion_points[i].kappa;
            double dx = evasion_points[i].x - evasion_points[i-1].x;
            double dy = evasion_points[i].y - evasion_points[i-1].y;
            el_lengths(i-1) = std::hypot(dx, dy);
        }

        // s 누적 벡터 계산
        auto [cur_layer, cur_idx] = nodeArray.front();
        
        std::vector<double> s_vec(N);
        s_vec[0] = stMap[RL_S][cur_layer];   // 현재 위치 raceline s에서 시작
        double track_length = stMap[RL_S].back();
        for (int i = 1; i < N; ++i) {
            s_vec[i] = s_vec[i-1] + el_lengths(i-1);
                if (s_vec[i] >= track_length) {
                    s_vec[i] -= track_length; // wrap-around 처리
                }
        }

        // 속도 프로파일 생성
        auto [end_layer, end_idx] = nodeArray.back();
        double v_start = stMap[RL_VX][cur_layer];
        double v_end   = stMap[RL_VX][end_layer]; 

        static VpForwardBackward vp(8.0, 5.0, params.vel_max, params.gg_scale);
        vp.updateDynParameters(params.vel_max, params.gg_scale);

        Eigen::VectorXd vx = vp.calcVelProfile(kappa, el_lengths, v_start, v_end);
        Eigen::VectorXd ax = accelFromProfile(vx, el_lengths);

        // fill wpnts
        for (int i=0; i<N; ++i) {
            std::vector<double> xs = {evasion_points[i].x};
            std::vector<double> ys = {evasion_points[i].y};
            std::vector<double> s_hint = {s_vec[i]};

            auto result = converter->get_frenet(xs, ys, &s_hint);
            auto& d_vec = result.second;

            double psi = std::atan2(evasion_points[i].y_d, evasion_points[i].x_d);
            auto w = xypsi_to_wpnt(evasion_points[i].x, evasion_points[i].y,
                                s_vec[i], d_vec[0],
                                psi, kappa(i),
                                vx(i), ax(i), i);
            wpnts.wpnts.push_back(w);

            visualization_msgs::msg::Marker m = xyv_to_marker(evasion_points[i].x, evasion_points[i].y, vx(i), i);
            wpnts_mrks.markers.push_back(m);

            // RCLCPP_INFO(this->get_logger(),
                // "Wpnt[%d]: x=%.3f, y=%.3f, s=%.3f, d=%.3f, psi=%.3f, kappa=%.3f, vx=%.3f",
                // i,
                // evasion_points[i].x, evasion_points[i].y,
                // s_vec[i], d_vec[0],
                // psi, kappa(i), vx(i));
        }

        // RCLCPP_INFO(this->get_logger(),
        //     "[Timing] obs_filter=%.3f ms | node_filter=%.3f ms | graph_search=%.3f ms | computeSplines=%.3f ms | interpSpline=%.3f ms | velProfile=%.3f ms",
        //     (t1-t0)*1000.0, (t2-t1)*1000.0, (t3-t2)*1000.0, (t4-t3)*1000.0, (t5-t4)*1000.0, (t6-t5)*1000.0
        // );
        // RCLCPP_INFO(this->get_logger(), "-----------------------------");
        return {wpnts, wpnts_mrks};
    }


    visualization_msgs::msg::Marker xyv_to_marker(double x, double y, double v, int id) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.scale.x = 0.1; 
        m.scale.y = 0.1; 
        m.scale.z = 0.1;
        m.color.a = 1.0;
        m.color.b = 0.75; m.color.r = 0.75;
        if (from_bag) m.color.g = 0.75;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.ns = "otwpnts";
        m.id = id;
        m.pose.position.x = x; m.pose.position.y = y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        return m;
    }

    f110_msgs::msg::Wpnt xypsi_to_wpnt(
        double x, double y, double s, double d,
        double psi, double kappa,
        double v, double ax, int id)
    {
        f110_msgs::msg::Wpnt w;
        w.id = id;
        w.s_m = s;
        w.d_m = d;
        w.x_m = x;
        w.y_m = y;
        w.psi_rad = psi;
        w.kappa_radpm = kappa;
        w.vx_mps = v;
        w.ax_mps2 = ax;
        return w;
    }

    pair<DVector, DVector> computeBoundRight(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &width_r) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || width_r.empty()) {
            throw runtime_error("computeBoundRight() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_bound_r(len), y_bound_r(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_bound_r[i] = pos_x[i] + norm_x[i] * width_r[i];
            y_bound_r[i] = pos_y[i] + norm_y[i] * width_r[i];
        }
        
        return {x_bound_r, y_bound_r};

    }

    pair<DVector, DVector> computeBoundLeft(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &width_l) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || width_l.empty()) {
            throw runtime_error("computeBoundLeft() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_bound_l(len), y_bound_l(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_bound_l[i] = pos_x[i] - norm_x[i] * width_l[i];
            y_bound_l[i] = pos_y[i] - norm_y[i] * width_l[i];
        }
        
        return {x_bound_l, y_bound_l};

    }

    pair<DVector, DVector> computeRaceline(DVector &pos_x, DVector &pos_y,
                                            DVector &norm_x, DVector &norm_y,
                                            DVector &norm_l) {
        if (pos_x.empty() || pos_y.empty() || norm_x.empty() || norm_y.empty() || norm_l.empty()) {
            throw runtime_error("computeBoundRaceline() - Empty DVector !!");
        }

        int len = pos_x.size();
        DVector x_raceline(len), y_raceline(len);
        
        for (size_t i = 0; i < len; ++i) {
            x_raceline[i] = pos_x[i] + norm_x[i] * norm_l[i];
            y_raceline[i] = pos_y[i] + norm_y[i] * norm_l[i];
        }
        
        return {x_raceline, y_raceline};

    }

    DVector computeDeltaS(DVector &rl_s) {
        if (rl_s.empty()) {
            throw runtime_error("computeDeltaS() - Empty DVector !!");
        }

        int len = rl_s.size();
        DVector rl_ds(len);

        // 마지막 원소는 0
        for (size_t i = 0; i < len - 1; ++i) {
            rl_ds[i] = rl_s[i+1] - rl_s[i];
        }
        
        return rl_ds;

    }

    DVector computeHeading(DVector &x_raceline, DVector &y_raceline) {

        DVector psi;
        size_t N = x_raceline.size();
        psi.resize(N);

        // 닫힌 회로 가정. 예외 처리 필요
        double dx, dy;
        for (size_t i = 0; i < N; ++i) {
            
            if (i != N -1) {
                dx = x_raceline[i+1] - x_raceline[i];
                dy = y_raceline[i+1] - y_raceline[i];
            } else {
                dx = x_raceline[0] - x_raceline[N - 1];
                dy = y_raceline[0] - y_raceline[N - 1];
            } 
        psi[i] = atan2(dy, dx) - M_PI_2;
            
        normalizeAngle(psi[i]);

        }

        return psi;
    }

    void loadGlobalTrajectoryMap() {
        // DMap gtMap = readDMapFromCSV(fname);

        auto [rb_x, rb_y] = computeBoundRight(gtMap[POS_X], gtMap[POS_Y],
                                                gtMap[NORM_X], gtMap[NORM_Y],
                                                gtMap[WIDTH_R]);
        gtMap[RB_X] = rb_x;
        gtMap[RB_Y] = rb_y;

        auto [lb_x, lb_y] = computeBoundLeft(gtMap[POS_X], gtMap[POS_Y],
                                            gtMap[NORM_X], gtMap[NORM_Y],
                                            gtMap[WIDTH_L]);
        gtMap[LB_X] = lb_x;
        gtMap[LB_Y] = lb_y;

        auto [rl_x, rl_y] = computeRaceline(gtMap[POS_X], gtMap[POS_Y],
                                            gtMap[NORM_X], gtMap[NORM_Y],
                                            gtMap[NORM_L]);
        gtMap[RL_X] = rl_x;
        gtMap[RL_Y] = rl_y;

        DVector rl_ds = computeDeltaS(gtMap[RL_S]);
        gtMap[RL_dS] = rl_ds;

        converter = std::make_unique<FrenetConverter>(rl_x, rl_y, gtMap[RL_PSI]);

    }

    IVector sampleLayersFromRaceline(const DVector& kappaVector, const DVector& distVector) {
        // RCLCPP_INFO(this->get_logger(), "Reached sampleLayersFromRaceline!");

        IVector layerIndexesSampled;
        const size_t n = kappaVector.size();
        double cur_dist = 0.0;
        double next_dist = 0.0;
        double next_dist_min = 0.0;

        for (size_t i = 0; i < n; ++i) {
            // 곡선이면 최소 거리 갱신
            if ((cur_dist + distVector[i]) > next_dist_min && fabs(kappaVector[i]) > params.curve_thr) {
                next_dist = cur_dist;
            }

            // 다음 샘플링 지점 도달
            if ((cur_dist + distVector[i]) > next_dist) {
                layerIndexesSampled.push_back(static_cast<int>(i));
                if (fabs(kappaVector[i]) < params.curve_thr) {  // 직선 구간
                    next_dist += params.d_straight;
                } else {  // 곡선 구간
                    next_dist += params.d_curve;
                }

                next_dist_min = cur_dist + params.d_curve;
            }

            cur_dist += distVector[i];
        }

        RCLCPP_INFO(this->get_logger(), "[INFO] Total number of track layers: %zu", layerIndexesSampled.size());

        return layerIndexesSampled;
    }
    
    DMap createSampledTrajectoryMap(DMap gtMap) {
        DMap stMap;
        
        IVector layerIndexesSampled = sampleLayersFromRaceline(gtMap[RL_KAPPA], gtMap[RL_dS]);

        for (const auto& [key, vec] : gtMap) {
            for (int idx : layerIndexesSampled) {
            if (idx >= 0 && idx < vec.size()) {
                stMap[key].push_back(vec[idx]);
                } 
            }
        }

        stMap[RL_dS] = computeDeltaS(stMap[RL_S]);
        stMap[RL_PSI] = computeHeading(stMap[RL_X], stMap[RL_Y]);
        stMap[LB_PSI] = computeHeading(stMap[LB_X], stMap[LB_Y]);
        stMap[RB_PSI] = computeHeading(stMap[RB_X], stMap[RB_Y]);  
        // RCLCPP_INFO(this->get_logger(), "Finished createSampledTrajectoryMap!");
        return stMap;
    }

    auto createNodeMap(DMap &stMap) -> pair<NodeMap, IVector> {

        
        const int N = stMap[NORM_L].size();
        Vector2d node_pos;
        nodeMap.resize(N);    // N개 레이어 기준, nodeMap 벡터를 N 크기로 초기화 (각 레이어에 노드 저장)
        // RCLCPP_INFO(this->get_logger(), "Reached createNodeMap! start!");
        // layer 별로 loop 돈다. for 루프 안이 한 레이어 내에서 하는 작업 내용물.
        for (size_t i = 0; i < N; ++i){ 
            ::Node node_;
            // raceline이 layer 내에서 몇 번째 인덱스인지 확인. 이를 기준으로 node의 첫 번째 기준을 삼을 예정(s).
            int raceline_index = floor((stMap[WIDTH_L][i] + stMap[NORM_L][i] - params.veh_width / 2) / params.lat_resolution);
            nodeIndicesOnRaceline.push_back(raceline_index);
            // RCLCPP_INFO(this->get_logger(),
            // "i=%zu WIDTH_L=%.3f, NORM_L=%.3f, veh_width=%.3f, lat_res=%.6f => raceline_index=%d",
            // i, stMap[WIDTH_L][i], stMap[NORM_L][i], params.veh_width, params.lat_resolution, raceline_index);

            Vector2d ref_xy(stMap[POS_X][i], stMap[POS_Y][i]);    // 기준선에서의 위치
            Vector2d norm_vec(stMap[NORM_X][i], stMap[NORM_Y][i]);  // 기준선에서 수직한 노멀 벡터 따라 노드 배치
            
            double start_alpha = stMap[NORM_L][i] - raceline_index * params.lat_resolution;    // 제일 왼쪽 노드가 노멀 벡터를 따라 얼마나 떨어져 있는지
            int node_idx = 0;
            int num_nodes = (stMap[WIDTH_R][i] + stMap[WIDTH_L][i] - params.veh_width) / params.lat_resolution;  // num_nodes : 좌우 총 가능한 노드 수
            if (num_nodes == raceline_index) num_nodes++; 
            nodeMap[i].resize(num_nodes); 
            // RCLCPP_INFO(this->get_logger(), "nodeMap[%ld]'s num_nodes: %d", i, num_nodes);  
            // node별 loop 
            for (int idx = 0; idx < num_nodes; ++idx) {
                double alpha = start_alpha + idx * params.lat_resolution;
                // node의 좌표 계산.
                node_pos = ref_xy + alpha * norm_vec;

                node_.x = node_pos.x();
                node_.y = node_pos.y();      
                node_.raceline = (node_idx == raceline_index);
                // psi 재계산  
                double psi_interp;
                if (node_idx < raceline_index) {
                    
                    if (abs(stMap[LB_PSI][i] - stMap[RL_PSI][i]) >= M_PI) 
                    {   
                        double bl = stMap[LB_PSI][i] + 2 * M_PI * (stMap[LB_PSI][i] < 0);
                        double p = stMap[RL_PSI][i] + 2 * M_PI * (stMap[RL_PSI][i] < 0);
                        psi_interp = bl + (p - bl) * node_idx / raceline_index;          
                    }
                    else {
                        psi_interp = stMap[LB_PSI][i] + (stMap[RL_PSI][i] - stMap[LB_PSI][i]) * (node_idx+1) / raceline_index;
                    }
                    node_.psi = normalizeAngle(psi_interp);
                }
                else if (node_idx == raceline_index) {
                    psi_interp = stMap[RL_PSI][i];
                    node_.psi = psi_interp;
                }
                else {
                    int remain = num_nodes - raceline_index - 1;
                    double t = static_cast<double>(node_idx - raceline_index) / max(remain, 1);  // 0 ~ 1
                    psi_interp = stMap[RL_PSI][i] + t * (stMap[RB_PSI][i] - stMap[RL_PSI][i]);
                    node_.psi = normalizeAngle(psi_interp);
                }

                nodeMap[i][node_idx] = node_;
                ++node_idx;

            }

        }
        // RCLCPP_INFO(this->get_logger(), "Reached createNodeMap! nodeMap size=%zu", nodeMap.size());
        return {nodeMap, nodeIndicesOnRaceline};
    }

    bool checkInsideBounds(DMap &stMap, const Vector2d& pos, const float veh_width) {

        if (stMap.find(LB_X) == stMap.end() || 
        stMap.find(LB_Y) == stMap.end() ||
        stMap.find(RB_X) == stMap.end() || 
        stMap.find(RB_Y) == stMap.end()) {
        throw invalid_argument("Boundary keys are missing in stMap!");
    }

        int n = stMap[LB_X].size();
        MatrixXd bound_l(n,2);
        MatrixXd bound_r(n,2);
        for (int i = 0; i < n; ++i) {
            bound_l(i, 0) = stMap[LB_X][i];
            bound_l(i, 1) = stMap[LB_Y][i];

            bound_r(i, 0) = stMap[RB_X][i];
            bound_r(i, 1) = stMap[RB_Y][i];
        }
        
        MatrixXd centerline = (bound_l + bound_r) / 2;

        // 가장 가까운 segment 인덱스 찾기
        int closestIdx = -1;
        double min_dist2 = numeric_limits<double>::max();
        for (int i = 0; i < centerline.rows() - 1; ++i) {
            // segment 중심 계산
            Vector2d mid = (centerline.row(i) + centerline.row(i + 1)) / 2.0;
            double dist2 = (mid - pos).squaredNorm();
            if (dist2 < min_dist2) {
                min_dist2 = dist2;
                closestIdx = i;
            }
        }

        if (closestIdx < 0 || closestIdx >= bound_l.rows() - 1)
            return false; // 예외 처리

        // bound_l, bound_r, centerline 보간 (선형 보간 10개 지점)
        int interp_points = 10;
        MatrixXd bl_interp(interp_points, 2);
        MatrixXd br_interp(interp_points, 2);
        MatrixXd center_interp(interp_points, 2);

        for (int i = 0; i < interp_points; ++i) {
            double t = static_cast<double>(i) / (interp_points - 1);
            bl_interp.row(i) = (1 - t) * bound_l.row(closestIdx) + t * bound_l.row(closestIdx + 1);
            br_interp.row(i) = (1 - t) * bound_r.row(closestIdx) + t * bound_r.row(closestIdx + 1);
            center_interp.row(i) = (1 - t) * centerline.row(closestIdx) + t * centerline.row(closestIdx + 1);
        }

        // pos에 가장 가까운 center_interp 인덱스 찾기
        int nearest_idx = -1;
        double best_dist2 = numeric_limits<double>::max();
        for (int i = 0; i < interp_points; ++i) {
            double d2 = (center_interp.row(i) - pos.transpose()).squaredNorm();
            if (d2 < best_dist2) {
                best_dist2 = d2;
                nearest_idx = i;
            }
        }

        // bound 사이 거리 (제곱)
        double d_track2 = (bl_interp.row(nearest_idx) - br_interp.row(nearest_idx)).squaredNorm();

        // 차량에서 각 bound까지 거리 (제곱)
        double d_bl_2 = (bl_interp.row(nearest_idx) - pos.transpose()).squaredNorm();
        double d_br_2 = (br_interp.row(nearest_idx) - pos.transpose()).squaredNorm();

        double dist_to_left_bound = sqrt(d_bl_2);
        double dist_to_right_bound = sqrt(d_br_2);

        // VEH_WIDTH 조건 확인
        if (dist_to_left_bound < veh_width || dist_to_right_bound < veh_width)
        {
            // throw invalid_argument("Spline point violates VEH_WIDTH constraints!");
            return false;
        }

        // bound 밖에 있는지 여부 확인
        bool within_bounds = !(d_bl_2 > d_track2 || d_br_2 > d_track2);
        return within_bounds;
    }

    IPair getClosestNodes(const Vector2d& pos, int limit=1) {
        IPair closestIdx;
        int num_nodes = 0;
        for (const auto& layer : nodeMap) {
            num_nodes += layer.size();
        }

        MatrixXd node_xy(num_nodes, 2);
        int idx = 0;

        for (size_t i = 0; i < nodeMap.size(); ++i) {
            for (size_t j = 0; j < nodeMap[i].size(); ++j) {
                const ::Node& node = nodeMap[i][j];
                node_xy(idx, 0) = node.x;
                node_xy(idx, 1) = node.y;
                ++idx;
            }
        }   
        // pos(2, 1) -> pos.transpose() -> (1, 2)
        MatrixXd diff = node_xy.rowwise() - pos.transpose();
        VectorXd dist2 = diff.rowwise().squaredNorm();
        vector<tuple<double, int, int>> dist_info;

        int re_idx = 0;
        for (size_t i = 0; i < nodeMap.size(); ++i) {
            for (size_t j = 0; j < nodeMap[i].size(); ++j) {
                dist_info.emplace_back(dist2(re_idx++), i, j);
            }
        }

        // 최소 거리 limit개만 앞으로 정렬
        nth_element(dist_info.begin(), dist_info.begin() + limit, dist_info.end());

        // 결과 저장
        for (int k = 0; k < limit; ++k) {
            auto [dist, i, j] = dist_info[k];
            closestIdx = make_pair(i, j);
            // RCLCPP_INFO(this->get_logger(), "Closest node: layer=%d, idx=%d", i, j);
        }
        return closestIdx;
    }

    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Online Loop /////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    void online_loop() {
        auto start_time = std::chrono::high_resolution_clock::now();
        // double wall_start = get_wall_time();
        // double cpu_start  = get_cpu_time();
        if (s_hint.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for s_hint...");
            return; // 아직 준비 안 됨
        }

        mpcc_ros::msg::Corridor corridor_array;
        visualization_msgs::msg::MarkerArray corridor_mrks;
        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray wpnts_mrks;
        
        // 마커 지우개
        visualization_msgs::msg::Marker del;
        del.header.stamp = this->now();
        del.header.frame_id = "map";   // frame도 반드시 지정
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        visualization_msgs::msg::MarkerArray clear;
        clear.markers.push_back(del);

        corridor_mrks_pub->publish(clear);
        wpnts_mrks_pub->publish(clear);

        // main online 단계 실행
        std::tie(wpnts, wpnts_mrks, corridor_array, corridor_mrks) = runOnline();
        last_switch_time = this->now();
        last_corridor = corridor_array; 

        wpnts.header.stamp = this->now();
        wpnts.header.frame_id = "map";

        // 토픽 발행
        evasion_pub->publish(wpnts);
        wpnts_mrks_pub->publish(wpnts_mrks);
        corridor_pub->publish(corridor_array);
        corridor_mrks_pub->publish(corridor_mrks);

        if (measuring) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            std_msgs::msg::Float32 latency;
            latency.data = static_cast<float>(elapsed.count());
            latency_pub->publish(latency);
        }


        // CPU 사용량 측정
        // double wall_end = get_wall_time();
        // double cpu_end  = get_cpu_time();

        // double wall_elapsed = wall_end - wall_start;
        // double cpu_elapsed  = cpu_end - cpu_start;

        // double cpu_usage = (cpu_elapsed / wall_elapsed) * 100.0;

        // std::cout << "Wall time: " << wall_elapsed << " sec\n";
        // std::cout << "CPU time : " << cpu_elapsed << " sec\n";
        // std::cout << "CPU usage: " << cpu_usage  << " %\n";
    }

f110_msgs::msg::Obstacle predict_obs_movement(f110_msgs::msg::Obstacle obs) {

    // double front_dist = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

    // if (front_dist < closest_obs_) {
    //     double delta_s = 0.0, delta_d = 0.0;
    //     double ot_distance = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

    //     int idx = std::min<int>(
    //         std::max<int>(0, static_cast<int>(cur_s * 10)),
    //         static_cast<int>(ltpl_wpnts_msg.ltplwpnts.size()) - 1
    //     );

    //     // 상대 속도 (내 racetraj 속도 - 상대 속도)
    //     double rel_speed = cur_vs - obs.vs;

    //     // 상대가 더 빠른 경우 → 나는 추월당하는 상황
    //     if (rel_speed <= 0.0) {
    //         // 이미 멀어지고 있으니 예측 보정 안 함
    //         return obs;
    //     }

    //     // 내가 더 빠른 경우 → 추월하는 상황
    //     // 예측 시간 계산 (최대 5초까지)
    //     double ot_time_distance = std::clamp(ot_distance / std::max(rel_speed, 0.1), 0.0, 2.0) * 0.5;

    //     // 위치 보정
    //     // dynamic parameter: obs_delay_s
    //     // dynamic parameter: obs_delay_d
    //     delta_s = ot_time_distance * obs.vs + obs_delay_s_; // 뒤쪽으로 offset
    //     delta_d = ot_time_distance * obs.vd + obs_delay_d_; // 좌우로 보정

    //     // s 업데이트
    //     obs.s_start = fmod(obs.s_start + delta_s + wpnt_max_s, wpnt_max_s);
    //     obs.s_center = fmod(obs.s_center + delta_s + wpnt_max_s, wpnt_max_s);
    //     obs.s_end   = fmod(obs.s_end   + delta_s + wpnt_max_s, wpnt_max_s);

    //     // d 업데이트
    //     obs.d_left   += delta_d;
    //     obs.d_center += delta_d;
    //     obs.d_right  += delta_d;

        // 디버그 마커 발행
        visualization_msgs::msg::Marker zone;
        zone.header.frame_id = "map";
        zone.header.stamp = this->now();
        zone.ns = "predicted_obs_zone";
        zone.id = obs.id;
        zone.type = visualization_msgs::msg::Marker::LINE_STRIP;
        zone.action = visualization_msgs::msg::Marker::ADD;
        zone.scale.x = 0.05; // 선 두께
        zone.color.a = 1.0;
        zone.color.r = 1.0; zone.color.g = 0.0; zone.color.b = 0.0;

        // s,d 좌표 → x,y 변환
        DVector s_vec = {obs.s_start, obs.s_start, obs.s_end, obs.s_end};
        DVector d_vec = {obs.d_left,  obs.d_right, obs.d_right, obs.d_left};
        auto resp = converter->get_cartesian(s_vec, d_vec);

        // 네 모서리 점
        for (size_t i = 0; i < resp.first.size(); i++) {
            geometry_msgs::msg::Point p;
            p.x = resp.first[i];
            p.y = resp.second[i];
            p.z = 0.0;
            zone.points.push_back(p);
        }
        // 닫아주기 (첫 점 다시 push)
        zone.points.push_back(zone.points.front());

        pub_propagated->publish(zone);

    // }
    return obs;
}

    std::vector<f110_msgs::msg::Obstacle> obs_filtering() {
        std::vector<f110_msgs::msg::Obstacle> obs_on_traj;
        
        // dynamic parameter: obs_traj_tresh_ -> raceline에 가까운지 판단하는 기준 offset
        // raceline에 붙어있지 않으면 회피할 필요X
        for (const auto & obs : obs_msg.obstacles) {
            if (std::abs(obs.d_center) < obs_traj_tresh_) obs_on_traj.push_back(obs);
        }

        // RCLCPP_INFO(this->get_logger(), "Total obstacles: %zu", obstacles.obstacles.size());
        // RCLCPP_INFO(this->get_logger(), "On traj obstacles: %zu", obs_on_traj.size());

        std::vector<f110_msgs::msg::Obstacle> close_obs;
        for (auto obs : obs_on_traj) {
            double dist = 0.0;
            obs = predict_obs_movement(obs);
            if (cur_s > obs.s_center) {
                dist = std::min(cur_s - obs.s_center, wpnt_max_s - cur_s + obs.s_center);
            }
            else {
                dist = std::min(obs.s_center - cur_s, wpnt_max_s - obs.s_center + cur_s);
            }

            // RCLCPP_INFO(this->get_logger(), "Candidate obs_s: %.2f, cur_s: %.2f, dist=%.2f",
                        // obs.s_center, cur_s, dist);
            // dynamic parameter: obs_lookahead_ -> 움직임 예측 후 보정된 obs 기준 회피 경로를 생성을 해야될 거리에 있는지(s값)
            if (dist < obs_lookahead_) close_obs.push_back(obs); //obs_lookahead_: 회피경로를 생성할 최대 예측 후의 장애물과 나와의 거리
        }

        // RCLCPP_INFO(this->get_logger(), "Obs_filtering got %zu close obs", close_obs.size());
        // RCLCPP_INFO(this->get_logger(), "------------------------------------------------");
        
        return close_obs;
    }
    
    std::vector<std::tuple<int,int,int>> find_obs_zone(
        const f110_msgs::msg::Obstacle &target_obs) {
        // TODO! getClosestNodes가 (s, d) 기준이면 이 과정이 없어도 된다. 
        std::vector<std::tuple<int,int,int>> blocked;

        auto obs_point_1 = converter->get_cartesian(target_obs.s_start, target_obs.d_left);
        auto obs_point_2 = converter->get_cartesian(target_obs.s_start, target_obs.d_right);
        auto obs_point_3 = converter->get_cartesian(target_obs.s_end,   target_obs.d_left);
        auto obs_point_4 = converter->get_cartesian(target_obs.s_end,   target_obs.d_right);
    
        Eigen::Vector2d obs_vec1(obs_point_1.first, obs_point_1.second);
        Eigen::Vector2d obs_vec2(obs_point_2.first, obs_point_2.second);
        Eigen::Vector2d obs_vec3(obs_point_3.first, obs_point_3.second);
        Eigen::Vector2d obs_vec4(obs_point_4.first, obs_point_4.second);

        IPair obs_front_l = getClosestNodes(obs_vec1);
        IPair obs_front_r = getClosestNodes(obs_vec2);
        IPair obs_back_l  = getClosestNodes(obs_vec3);
        IPair obs_back_r  = getClosestNodes(obs_vec4);

        // front랑 back 노드 인덱스가 같은 경우
        if (obs_front_l == obs_front_r) {
        // 좌우가 같은 노드라면 인덱스를 조금 퍼뜨려줌(수정 필요. +1한 상태에서 해당 노드가 없을 수 있음.)
        obs_front_l.second = std::max(0, obs_front_l.second - 1);
        obs_front_r.second = std::min((int)nodeMap[obs_front_r.first].size()-1,
                                    obs_front_r.second + 1);
        }
        if (obs_back_l == obs_back_r) {
            obs_back_l.second = std::max(0, obs_back_l.second - 1);
            obs_back_r.second = std::min((int)nodeMap[obs_back_r.first].size()-1,
                                        obs_back_r.second + 1);
        }

            int start_layer = std::max(obs_front_l.first, obs_front_r.first);
            int end_layer   = std::min(obs_back_l.first,  obs_back_r.first);
            if (start_layer == end_layer) {
                if (start_layer < (int)nodeMap.size()-1) {
                    end_layer = start_layer + 1; // 뒤쪽으로 한 레이어 확장
                } else if (start_layer > 0) {
                    start_layer = start_layer - 1; // 앞쪽으로 한 레이어 확장
                }
            }

        // front 기준 lateral index 범위
        int idx_min = std::max(0, std::min(obs_front_l.second, obs_front_r.second) - inflate_idx_);
        int idx_max = std::min((int)nodeMap[start_layer].size()-1,
                            std::max(obs_front_l.second, obs_front_r.second) + inflate_idx_);

        // start_layer ~ end_layer 전부 push
        for (int l = start_layer; l <= end_layer; ++l) {
            blocked.push_back(std::make_tuple(l, idx_min, idx_max));
            // RCLCPP_INFO(rclcpp::get_logger("corridor_generator"),
            //             "Blocked zone: layer=%d, idx_min=%d, idx_max=%d",
            //             l, idx_min, idx_max);
        }

        // RCLCPP_INFO(rclcpp::get_logger("corridor_generator"),
        //             "find_obs_jone -> total blocked ranges: %zu", blocked.size());

        return blocked;
    }

    int getClosestLayer(double target_s) {
        const auto& s_vec = stMap.at(RL_S);
        if (s_vec.empty()) {
            throw std::runtime_error("stMap[RL_S] is empty!");
        }

        auto it = std::lower_bound(s_vec.begin(), s_vec.end(), target_s);
        if (it == s_vec.begin()) return 0;
        if (it == s_vec.end()) return s_vec.size() - 1;

        int idx = it - s_vec.begin();
        // 이전 값과 비교해서 더 가까운 쪽 선택
        // target_s보다 크거나 같은 layer
        if (std::abs(s_vec[idx-1] - target_s) < std::abs(s_vec[idx] - target_s)) {
            return idx-1;
        } else {
            return idx;
        }
    }

    auto findDestination() -> pair<IPair, IPair> {

        Eigen::Vector2d cur_xy(cur_x, cur_y);

        IPair startIdx = getClosestNodes(cur_xy, 1);

        double target_s = s_hint.back();
        
        // planning horizon의 끝지점 s와 같거나 큰 레이어
        int dest_layer = getClosestLayer(target_s);

        if (dest_layer >= (int)nodeIndicesOnRaceline.size()) 
            dest_layer = dest_layer % nodeIndicesOnRaceline.size();
        
        // 일단 목표 지점은 dest_layer의 raceline node 
        int dest_index = nodeIndicesOnRaceline[dest_layer];
        IPair endIdx = {dest_layer, dest_index};

        cout << "[findDestination] start=(L" << startIdx.first << ", i" << startIdx.second
              << "), dest=(L" << endIdx.first << ", i" << endIdx.second 
              << "), target_s=" << target_s << endl;
            
        return {startIdx, endIdx};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorGenerator>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}