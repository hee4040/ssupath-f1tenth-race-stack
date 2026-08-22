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
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/ltpl_wpnt.hpp"
#include "f110_msgs/msg/ltpl_wpnt_array.hpp"

#include "graph_planner.hpp"
#include "NodeGraph.hpp"
#include "offline_params.hpp"
#include "velocity_profile.hpp"
#include "frenet_conversion_cpp/frenet_converter_cpp.hpp"
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <map>
#include <set>

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

// using std:::placeholders:_1;

class ObstacleSpliner : public rclcpp::Node {
public:
    ObstacleSpliner(): Node("graph_planner"), planning_done(false) {
        // offline parameter
        params = load_offline_params(this);
        // online parameter
        this->declare_parameter<double>("obs_delay_d", 0.0);
        this->declare_parameter<double>("obs_delay_s", 0.0);
        // inflate_idx: find_obs_jone 에서 물리적으로 계산한 팽창량 위에 "추가로" 얹는 여유 노드 수.
        //   예전에는 이 값이 유일한 횡방향 팽창량이었고 6 (= 0.6 m) 이었다. lobby 트랙은
        //   통행 가능 폭이 중앙값 1.2 m (노드 12개) 밖에 안 되므로 장애물 1개로 layer 전체가
        //   막혀버렸다 -> 자유 노드가 없어 회피해가 사라졌다. 물리량 기반 계산으로 대체.
        this->declare_parameter<int>("inflate_idx", 0);
        this->declare_parameter<int>("min_plan_horizon", 11);
        this->declare_parameter<double>("obs_traj_tresh", 2.5);
        this->declare_parameter<double>("closest_obs", 2.0);
        this->declare_parameter<double>("obs_lookahead", 8.0);
        // safety_margin: ego 반차폭에 더해지는 횡방향 여유 [m]. 폭이 충분할 때 확보하는 목표값.
        //   차단 여유 = veh_width/2 + safety_margin + inflate_idx * lat_resolution
        this->declare_parameter<double>("safety_margin", 0.15);
        // min_safety_margin: 폭이 좁아 해가 없을 때 layer 단위로 여기까지 여유를 깎는다 [m].
        //   0 이면 "차체가 닿지만 않으면 된다"는 물리 하한. 완화를 끄려면 safety_margin 과 같게 둔다.
        //   state_machine 의 lateral_width_ot_m 은 반드시
        //   veh_width/2 < lateral_width_ot_m < veh_width/2 + min_safety_margin 이어야 한다.
        //   (크면 여기서 완화해 만든 정상 경로를 state machine 이 도로 거부한다)
        //   veh_width 0.28 기준 -> 0.14 < lateral_width_ot_m < 0.19, 현재 0.17.
        this->declare_parameter<double>("min_safety_margin", 0.05);
        // block_obs_tail_layer (2026-08-20): 장애물이 걸친 layer 바로 뒤 한 장("꼬리 layer")도
        //   같이 막을지. 예전 거동은 true 였다(find_obs_jone 의 span+1). 그 layer 에는 장애물의
        //   frenet d 밴드가 그대로 복사되는데, 코너에서는 raceline 이 옆으로 이동하므로 밴드가
        //   엉뚱한 쪽을 지워 통로를 좌우로 끊었다. 장애물을 뚫는 edge 는 진짜 layer 의 차단
        //   노드가 이미 잡으므로 기본값 false. 문제가 생기면 true 로 되돌린다.
        this->declare_parameter<bool>("block_obs_tail_layer", false);
        // hard_block_nodes (2026-08-20): 장애물이 가리는 노드를 graph_search 에서 아예 제외할지.
        //   false 면 예전 방식(apply_node_filter 로 나가는 edge 에 +1e6 벌점).
        //   벌점은 유한해서 우회 곡률 비용이 1e6 을 넘으면 "뚫고 가는 해"가 최적이 되고,
        //   runOnline 이 그걸 통째로 폐기해 빈 경로가 나갔다(obs_debug_0820_1348 실측 73.6%).
        //   true 면 그런 해가 애초에 생성되지 않는다 -> 나온 경로는 항상 안전, 못 찾으면 진짜 없음.
        this->declare_parameter<bool>("hard_block_nodes", true);

        // ---- 스플라인 충돌 검사 (2026-08-22) ----------------------------
        // 노드 차단은 '검문소(layer)에서 어느 칸을 고를지' 만 제약한다. 실제로 발행되는
        // 것은 그 노드들을 이은 스플라인 위의 샘플점이고, 그 곡선 자체는 아무도 보지
        // 않았다. 레이어 간격(hall_0822 실측 0.80 m)이 장애물 크기(0.34~0.41 m)보다
        // 크기 때문에, 양 끝 노드가 자유여도 사이 구간이 장애물을 지날 수 있다.
        //
        // 실측 (obs_debug_0822_1431, 419 s):
        //   발행된 otwpnts 1209개 중 155개(12.8%)가 차체 반폭 여유조차 없었다.
        //   그중 85%가 '첫 엣지'(경로 시작~0.80 m) 안에서 일어났고,
        //   65.9%는 경로의 첫 점부터 이미 장애물 안이었다.
        //   원인: findDestination 이 시작 노드를 차의 현재 (s,d) 로 잡으면서 그 노드가
        //   차단 노드인지 보지 않고, 아래 PATH_BLOCKED 검사도 index 0 을 면제한다.
        //   layer_at_s 가 '최근접' 레이어를 돌려주므로 장애물이 차 앞 반 레이어 안에
        //   있으면 차 자신의 레이어에 배정되고 = 그게 곧 면제되는 레이어다.
        this->declare_parameter<bool>("spline_check", true);
        // 충돌이 나면 그 구간 하류 노드를 빼고 몇 번까지 다시 풀지.
        // 0 이면 재탐색 없이 바로 빈 경로(= 예전 PATH_BLOCKED 거동).
        // 실측상 침범 상황의 82.1% 는 좌우 어느 한쪽에 통과 가능한 공간이 있었으므로
        // 재탐색이 대부분 대안을 찾는다. 나머지 17.9% 는 실제로 폭이 부족한 경우다.
        this->declare_parameter<int>("spline_check_retries", 3);
        // 검사를 면제할 '차 앞' 거리 [m]. 0 이하면 veh_length/2 를 쓴다.
        // 이 거리 안의 충돌은 이미 피할 수 없는 것이라, 버려도 대안이 나오지 않고
        // 경로만 사라진다(그 구간의 대응은 감속이지 조향이 아니다).
        this->declare_parameter<double>("spline_check_skip_m", 1.0);
        // ---- 속도 비례 safety_margin (2026-08-04 신규) ----------------------
        // 고속일수록 pose 오차와 추종 오차가 커지므로 회피선을 장애물에서 더 멀리 돌린다.
        // obs_debug_0804_2103 실측: 나란히 지나간 1019 프레임의 표면 여유 중앙이
        // +0.102 m 뿐이고 27.8% 가 음수(스침)였다. 그 접촉 건들은 rel 중앙 0.25 로
        // 트랙 한가운데라 유령이 아니라 진짜 근접 통과다.
        //   cur_vs <= sm_v_lo  -> safety_margin      (저속: 기존 그대로)
        //   cur_vs >= sm_v_hi  -> safety_margin_hi
        //   사이               -> 선형 보간
        // safety_margin_hi 를 safety_margin 이하로 두면 기능이 사실상 꺼진다.
        // 좁은 구간에서는 어차피 layer 단위로 min_safety_margin 까지 자동 완화되므로
        // 이 값을 올려도 "회피 자체가 불가능해지는" 일은 잘 생기지 않는다.
        this->declare_parameter<double>("safety_margin_hi", 0.15);
        this->declare_parameter<double>("sm_v_lo", 2.0);
        this->declare_parameter<double>("sm_v_hi", 4.5);
        this->declare_parameter<double>("hyst_time", 0.0);
        this->declare_parameter<double>("overtake_speed_gain", 1.2);
        // ---- 정지 원인 진단 로그 (2026-08-20) --------------------------------
        // 차가 멈춰 있는데 회피 경로가 안 나올 때 "왜" 를 로그에 남긴다.
        // 계산은 아래 두 곳에서만 일어난다:
        //   (a) runOnline 의 실패 return 마다 snapshot 기록 (정수/실수 몇 개 + zones 복사)
        //   (b) report_stall() - 차가 실제로 멈춰 있고 장애물이 보일 때, 주기당 1회
        // 정상 주행 중에는 (a) 조차 돌지 않으므로 루프 비용은 사실상 0 이다.
        // layer_lookup_by_s (2026-08-20): 레이어를 (x,y) 최근접 대신 s 로 찾을지.
        //   false = 예전 방식(getClosestNodes). 코너 안쪽/접힘 구간에서 엉뚱한 레이어로
        //   스냅될 수 있고, find_obs_jone 은 모서리 4개의 min/max 를 쓰기 때문에 하나만
        //   튀어도 장애물 하나가 레이어를 무더기로 막는다.
        //   ★ 2026-08-20 실측 결과 기본값을 false 로 둔다. lobby_0820(obs_debug_0820_2043,
        //   200~300 s) 에서 두 방식을 나란히 계산해 비교했더니 시작노드 레이어 불일치가
        //   996회 중 4회(0.4%), 최대 차이 1 레이어였고, 장애물 존의 최대 span 은 2 레이어였다.
        //   즉 이 맵에서는 오스냅이 실질적으로 없어 s 기반으로 바꿔도 얻는 게 없다.
        //   (헤어핀이 심한 맵에서는 다를 수 있으므로 코드와 진단은 남겨 둔다. 켜려면 true)
        this->declare_parameter<bool>("layer_lookup_by_s", false);
        // snap_diag: 두 방식을 매 주기 나란히 계산해 불일치를 집계한다(진단용).
        this->declare_parameter<bool>("snap_diag", true);
        this->declare_parameter<bool>("stall_diag", true);
        this->declare_parameter<double>("stall_speed_thresh", 0.15);  // [m/s] 이하를 '정지'로 본다
        this->declare_parameter<double>("stall_report_after", 1.5);   // [s] 정지가 이만큼 지속되면 보고 시작
        this->declare_parameter<double>("stall_report_period", 2.0);  // [s] 보고 간격

        // ---- 회피 방향 잠금 (2026-08-21) ------------------------------------
        // 왜 필요한가: obs_debug_0821_1417(495초) 실측에서 장애물 통과 67회 중 16회가
        // 표면 여유 0.14 m 미만(= 차 몸통이 닿는 거리)으로 지나갔다. 원인은 회피 방향이
        // 마지막 순간까지 바뀌는 것이다. 이 차의 실측 횡방향 이동 능력은 p90 0.63 m/s 라
        // 0.4 m 비키는 데 0.63초 = 3 m/s 에서 1.9 m 를 전진한다. 즉 장애물이 2 m 안에
        // 들어온 뒤의 방향 변경은 물리적으로 실행 불가능하고, 좌우 명령을 번갈아 받은 차는
        // 어느 쪽으로도 못 가고 가운데(=장애물)로 간다.
        //
        // 잠그는 대상은 '경로'가 아니라 '어느 쪽으로 피할지' 하나다. 경로 기하는 매 주기
        // 새로 풀되, 대상 장애물의 layer 에서 반대쪽 노드를 후보에서 제외해 방향만 유지한다.
        // (hard_block_nodes 와 같은 하드 제외 방식이다. hysteresisBias 의 +1e6 벌점 방식은
        //  벌점이 유한해 우회 비용이 그걸 넘으면 뚫고 가는 해가 최적이 되므로 쓰지 않는다.)
        this->declare_parameter<bool>("side_lock", true);
        // [m] 대상 장애물이 이 거리 안에 들어오면 방향 변경을 금지한다.
        //   이 안에서 잠근 쪽 해가 없으면 방향을 바꾸는 게 아니라 빈 경로를 낸다
        //   -> state machine 이 TRAILING 으로 감속한다(= "방향 대신 속도를 줄인다").
        //   실측 근거: 0.4 m 횡이동에 0.63초, v=2.5 m/s 에서 1.58 m, v=3.0 에서 1.90 m.
        this->declare_parameter<double>("side_freeze_dist", 1.7);
        // 동결 거리 밖에서, 잠근 쪽으로 해가 이만큼 연속 실패하면 잠금을 푼다.
        // 1 로 두면 한 프레임 흔들림에도 방향이 바뀌어 잠금이 무의미해진다.
        this->declare_parameter<int>("side_unlock_fails", 3);
        // [m] 대상 장애물이 '같은 장애물'인지 판정하는 s 허용오차. 이보다 크게 튀면
        //   새 장애물로 보고 잠금을 푼다(새 장애물은 새 정보이므로 동결 거리와 무관).
        this->declare_parameter<double>("side_lock_match_dist", 1.0);

        obs_delay_s_    = this->get_parameter("obs_delay_s").as_double();
        obs_delay_d_        = this->get_parameter("obs_delay_d").as_double();
        min_plan_horizon_   = this->get_parameter("min_plan_horizon").as_int();
        inflate_idx_        = this->get_parameter("inflate_idx").as_int();
        obs_traj_tresh_     = this->get_parameter("obs_traj_tresh").as_double();
        closest_obs_        = this->get_parameter("closest_obs").as_double();
        obs_lookahead_      = this->get_parameter("obs_lookahead").as_double();
        safety_margin_      = this->get_parameter("safety_margin").as_double();
        min_safety_margin_  = this->get_parameter("min_safety_margin").as_double();
        block_obs_tail_layer_ = this->get_parameter("block_obs_tail_layer").as_bool();
        hard_block_nodes_     = this->get_parameter("hard_block_nodes").as_bool();
        spline_check_         = this->get_parameter("spline_check").as_bool();
        spline_check_retries_ = this->get_parameter("spline_check_retries").as_int();
        spline_check_skip_m_  = this->get_parameter("spline_check_skip_m").as_double();
        safety_margin_hi_   = this->get_parameter("safety_margin_hi").as_double();
        sm_v_lo_            = this->get_parameter("sm_v_lo").as_double();
        sm_v_hi_            = this->get_parameter("sm_v_hi").as_double();
        hyst_time_          = this->get_parameter("hyst_time").as_double();
        overtake_speed_gain_ = this->get_parameter("overtake_speed_gain").as_double();
        layer_lookup_by_s_   = this->get_parameter("layer_lookup_by_s").as_bool();
        snap_diag_           = this->get_parameter("snap_diag").as_bool();
        stall_diag_          = this->get_parameter("stall_diag").as_bool();
        stall_speed_thresh_  = this->get_parameter("stall_speed_thresh").as_double();
        stall_report_after_  = this->get_parameter("stall_report_after").as_double();
        stall_report_period_ = this->get_parameter("stall_report_period").as_double();
        side_lock_           = this->get_parameter("side_lock").as_bool();
        side_freeze_dist_    = this->get_parameter("side_freeze_dist").as_double();
        side_unlock_fails_   = this->get_parameter("side_unlock_fails").as_int();
        side_lock_match_dist_= this->get_parameter("side_lock_match_dist").as_double();
        
        param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObstacleSpliner::paramCB, this, std::placeholders::_1));

        this->declare_parameter<bool>("from_bag", false);
        this->declare_parameter<bool>("measure", false);
        from_bag = this->get_parameter("from_bag").as_bool();
        measuring = this->get_parameter("measure").as_bool();
        
        rclcpp::QoS qos(10);
        qos.transient_local().reliable();

        // Subscriber - Offline
        ltpl_waypoints_sub = this->create_subscription<f110_msgs::msg::LtplWpntArray>(
            "/ltpl_waypoints", 10, std::bind(&ObstacleSpliner::ltpl_cb, this, std::placeholders::_1));

        // Subscriber - Online
        obs_sub = this->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/perception/obstacles", 10, std::bind(&ObstacleSpliner::obs_cb, this, std::placeholders::_1));
        state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/car_state/frenet/odom", 10, std::bind(&ObstacleSpliner::state_cb, this, std::placeholders::_1));

        // publisher 
        mrks_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/avoidance/markers", qos);
        evasion_pub = this->create_publisher<f110_msgs::msg::OTWpntArray>("/planner/avoidance/otwpnts", qos);
        pub_propagated = this->create_publisher<visualization_msgs::msg::Marker>("/planner/avoidance/propagated_obs", qos);

        if (measuring) {
            latency_pub = this->create_publisher<std_msgs::msg::Float32>("/graph_planner/avoidance/latency", qos);
        }

        // Wait for initial messages (blocking-like behavior but safe)
        RCLCPP_INFO(this->get_logger(), "Waiting for initial messages...");
        wait_for_initial_messages();

        // Offline Part run -> once
        if (!planning_done) runOffline();

        // create timer at 20 Hz
        timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleSpliner::online_loop, this));
    }
    
private:
    // Subscriber
    rclcpp::Subscription<f110_msgs::msg::LtplWpntArray>::SharedPtr ltpl_waypoints_sub;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrks_pub;
    rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr evasion_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_propagated;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub;

    // OfflineParams params;
    OfflineParams params;  
    bool measuring{false};
    bool from_bag{false};
    bool planning_done;

    // last_switch_time/side
    //   last_switch_time 은 "회피 방향이 실제로 바뀐 시각"이다. 이 값은 OTWpntArray 메시지에
    //   실려 state machine 의 splini_hyst_timer_sec 판정에 쓰인다. 2026-08-21 이전에는
    //   메시지에 채워 넣는 코드가 없어서(멤버만 매 주기 갱신) 그 히스테리시스가 완전히
    //   죽어 있었다 - obs_debug_0821_1417 의 otwpnts 9919 개 전부 미설정, 발동 0회.
    rclcpp::Time last_switch_time{0,0,RCL_ROS_TIME};    
    string last_side;   

    // ---- 회피 방향 잠금 상태 (2026-08-21) ----
    bool   side_lock_{true};
    double side_freeze_dist_{1.7};
    int    side_unlock_fails_{3};
    double side_lock_match_dist_{1.0};
    // 잠근 방향. 0 = 없음, +1 = 장애물의 d 보다 큰 쪽(프레네 왼쪽), -1 = 작은 쪽(오른쪽).
    // 부호를 노드 인덱스가 아니라 프레네 d 로 정의해 d_sign_ 에 의존하지 않는다.
    int    locked_side_{0};
    double lock_obs_s_{0.0};     // 잠금이 가리키는 장애물의 s
    double lock_obs_d_{0.0};     // 그 장애물의 d (방향 판정 기준)
    int    lock_fail_{0};        // 잠근 쪽으로 연속 실패한 횟수
    bool   published_side_valid_{false};
    int    published_side_{0};   // 직전에 실제로 발행한 경로의 방향

    // state variables
    double cur_s{0.0}, cur_d{0.0}, cur_vs{0.0};
    double wpnt_max_s{0.0};
    int inflate_idx_{2};
    int min_plan_horizon_;
    double obs_traj_tresh_;
    double closest_obs_;
    bool   spline_check_{true};
    int    spline_check_retries_{3};
    double spline_check_skip_m_{0.0};
    double obs_lookahead_;
    double obs_delay_d_;
    double safety_margin_;
    double min_safety_margin_{0.0};
    double safety_margin_hi_{0.15};
    double sm_v_lo_{2.0};
    double sm_v_hi_{4.5};
    double obs_delay_s_;
    double hyst_time_;
    double overtake_speed_gain_;
    // 장애물 뒤 "꼬리 layer" 도 차단할지 (기본 false, find_obs_jone 참조)
    bool   block_obs_tail_layer_{false};
    // 차단 노드를 graph_search 에서 하드 제외할지 (기본 true, graph_search 주석 참조)
    bool   hard_block_nodes_{true};
    // 정지 원인 진단 (아래 FailSnapshot / report_stall 참조)
    bool   stall_diag_{true};
    double stall_speed_thresh_{0.15};
    double stall_report_after_{1.5};
    double stall_report_period_{2.0};
    rclcpp::Time stall_since_{0,0,RCL_ROS_TIME};
    rclcpp::Time last_stall_report_{0,0,RCL_ROS_TIME};
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    DMap gtMap;
    DMap stMap;
    NodeMap nodeMap;
    IVector nodeIndicesOnRaceline;
    f110_msgs::msg::ObstacleArray obs_msg;
    f110_msgs::msg::LtplWpntArray ltpl_wpnts_msg;
    f110_msgs::msg::OTWpntArray last_wpnts;
    rclcpp::TimerBase::SharedPtr timer_;
    bool have_state{false}, have_ltpl{false};
    NodeGraph nodeGraph;
    std::string final_csv_path;
    std::unique_ptr<FrenetConverter> converter;
    double gb_vmax;
    // 노드 인덱스 증가 방향 <-> frenet d 부호. calibrate_d_sign() 이 실측으로 채운다.
    double d_sign_{-1.0};

    // ---- 레이어 조회 방식 및 진단 카운터 (2026-08-20) ----
    bool layer_lookup_by_s_{true};
    bool snap_diag_{true};
    long snap_n_{0}, snap_mismatch_{0}, snap_gap_sum_{0}, snap_gap_max_{0};
    long zone_n_{0}, zone_mismatch_{0}, zone_span_old_{0}, zone_span_new_{0}, zone_span_old_max_{0};
    rclcpp::Time snap_last_report_{0,0,RCL_ROS_TIME};

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
            } else if (p.get_name() == "safety_margin") {
                safety_margin_ = p.as_double();
                RCLCPP_INFO(get_logger(), "safety_margin updated: %.3f", safety_margin_);
            } else if (p.get_name() == "min_safety_margin") {
                min_safety_margin_ = p.as_double();
                RCLCPP_INFO(get_logger(), "min_safety_margin updated: %.3f", min_safety_margin_);
            } else if (p.get_name() == "block_obs_tail_layer") {
                block_obs_tail_layer_ = p.as_bool();
                RCLCPP_INFO(get_logger(), "block_obs_tail_layer updated: %s",
                            block_obs_tail_layer_ ? "true" : "false");
            } else if (p.get_name() == "hard_block_nodes") {
                hard_block_nodes_ = p.as_bool();
                RCLCPP_INFO(get_logger(), "hard_block_nodes updated: %s",
                            hard_block_nodes_ ? "true" : "false");
            } else if (p.get_name() == "spline_check") {
                spline_check_ = p.as_bool();
                RCLCPP_INFO(get_logger(), "spline_check updated: %s",
                            spline_check_ ? "true" : "false");
            } else if (p.get_name() == "spline_check_retries") {
                spline_check_retries_ = p.as_int();
                RCLCPP_INFO(get_logger(), "spline_check_retries updated: %d", spline_check_retries_);
            } else if (p.get_name() == "spline_check_skip_m") {
                spline_check_skip_m_ = p.as_double();
                RCLCPP_INFO(get_logger(), "spline_check_skip_m updated: %.3f", spline_check_skip_m_);
            } else if (p.get_name() == "layer_lookup_by_s") {
                layer_lookup_by_s_ = p.as_bool();
                RCLCPP_INFO(get_logger(), "layer_lookup_by_s updated: %d", (int)layer_lookup_by_s_);
            } else if (p.get_name() == "snap_diag") {
                snap_diag_ = p.as_bool();
            } else if (p.get_name() == "safety_margin_hi") {
                safety_margin_hi_ = p.as_double();
                RCLCPP_INFO(get_logger(), "safety_margin_hi updated: %.3f", safety_margin_hi_);
            } else if (p.get_name() == "sm_v_lo") {
                sm_v_lo_ = p.as_double();
                RCLCPP_INFO(get_logger(), "sm_v_lo updated: %.3f", sm_v_lo_);
            } else if (p.get_name() == "sm_v_hi") {
                sm_v_hi_ = p.as_double();
                RCLCPP_INFO(get_logger(), "sm_v_hi updated: %.3f", sm_v_hi_);
            } else if (p.get_name() == "obs_delay_s") {
                obs_delay_s_ = p.as_double();
                RCLCPP_INFO(get_logger(), "obs_delay_s updated: %.3f", obs_delay_s_);
            } else if (p.get_name() == "min_plan_horizon") {
                min_plan_horizon_ = p.as_int();
                RCLCPP_INFO(get_logger(), "min_plan_horizon updated: %d", min_plan_horizon_);
            } else if (p.get_name() == "hyst_time") {
                hyst_time_ = p.as_double();
                RCLCPP_INFO(get_logger(), "hyst_time updated: %.3f", hyst_time_);
            } else if (p.get_name() == "overtake_speed_gain") {
                overtake_speed_gain_ = p.as_double();
                RCLCPP_INFO(get_logger(), "overtake_speed_gain updated: %.3f", overtake_speed_gain_);
            } else if (p.get_name() == "side_lock") {
                side_lock_ = p.as_bool();
                if (!side_lock_) { locked_side_ = 0; lock_fail_ = 0; }
                RCLCPP_INFO(get_logger(), "side_lock updated: %s", side_lock_ ? "true" : "false");
            } else if (p.get_name() == "side_freeze_dist") {
                side_freeze_dist_ = p.as_double();
                RCLCPP_INFO(get_logger(), "side_freeze_dist updated: %.3f m", side_freeze_dist_);
            } else if (p.get_name() == "side_unlock_fails") {
                side_unlock_fails_ = p.as_int();
                RCLCPP_INFO(get_logger(), "side_unlock_fails updated: %d", side_unlock_fails_);
            } else if (p.get_name() == "side_lock_match_dist") {
                side_lock_match_dist_ = p.as_double();
                RCLCPP_INFO(get_logger(), "side_lock_match_dist updated: %.3f m", side_lock_match_dist_);
            }
        }
        return result;
    }

    // Callback
    void ltpl_cb(const f110_msgs::msg::LtplWpntArray::SharedPtr msg) {
        ltpl_wpnts_msg = *msg;
        try {
            gtMap.clear();
            for (const auto &wp : msg->ltplwpnts) {
                gtMap["x_ref_m"].push_back(wp.x_ref_m);
                gtMap["y_ref_m"].push_back(wp.y_ref_m);
                gtMap["width_right_m"].push_back(wp.width_right_m);
                gtMap["width_left_m"].push_back(wp.width_left_m);
                gtMap["x_normvec_m"].push_back(wp.x_normvec_m);
                gtMap["y_normvec_m"].push_back(wp.y_normvec_m);
                gtMap["alpha_m"].push_back(wp.alpha_m);
                gtMap["s_racetraj_m"].push_back(wp.s_racetraj_m);
                gtMap["psi_racetraj_rad"].push_back(wp.psi_racetraj_rad);
                gtMap["kappa_racetraj_radpm"].push_back(wp.kappa_racetraj_radpm);
                gtMap["vx_racetraj_mps"].push_back(wp.vx_racetraj_mps);
                gtMap["ax_racetraj_mps2"].push_back(wp.ax_racetraj_mps2);

                auto vmax_it = std::max_element(gtMap["vx_racetraj_mps"].begin(),
                                gtMap["vx_racetraj_mps"].end());
                double gb_vmax = (vmax_it != gtMap["vx_racetraj_mps"].end()) ? *vmax_it : 0.0;
                have_ltpl = true;
            }
            wpnt_max_s = msg->ltplwpnts.back().s_racetraj_m;

            // RCLCPP_INFO(this->get_logger(), "gtMap updated with %zu waypoints", msg->ltplwpnts.size());
            // RCLCPP_INFO(this->get_logger(), "Offline planning done, shutting down node.");

        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in offline planning %s", e.what());
            rclcpp::shutdown();
        }
        catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Unkown exception in offline planning.");
            rclcpp::shutdown();
        }
    }

    void obs_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
        obs_msg = *msg;
    }

    void state_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        cur_s = msg->pose.pose.position.x;
        cur_d = msg->pose.pose.position.y;
        cur_vs = msg->twist.twist.linear.x;
        have_state = true;
    }

    void wait_for_initial_messages() {
        rclcpp::Rate r(10);
        while (rclcpp::ok()) {
            if (have_state && have_ltpl) break;
            r.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
        }
        RCLCPP_INFO(this->get_logger(), "All required messages received. Continuing...");
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
            nodeGraph.pruneEdges(nodeMap, stMap[RL_VX]);
            nodeGraph.computeSplineCost(nodeIndicesOnRaceline);
 
            // if (!nodeGraph.writeSplineMapToCSV(paramsfinal_csv_path)) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to write CSV to path: %s", params.csv_output_path);
            //     throw std::runtime_error("CSV write failed");
            // }
            
            // 노드 인덱스 <-> frenet d 부호 판정 (차단 영역 계산에 필수)
            calibrate_d_sign();

            nodeGraph.printGraph(this->get_logger());
            final_csv_path = params.csv_output_path + params.map_name + "/SplineMap.csv";
            // nodeGraph.writeSplineMapToCSV(final_csv_path, this->get_logger());

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

    // 현재 속도에 맞는 safety_margin [m].
    // 저속(sm_v_lo 이하)에서는 기존 safety_margin 을 그대로 써서 거동을 보존한다.
    // safety_margin_hi <= safety_margin 이면 계산을 건너뛰어 기능이 꺼진 것과 같다.
    double effective_safety_margin() const {
        if (safety_margin_hi_ <= safety_margin_) return safety_margin_;
        if (sm_v_hi_ <= sm_v_lo_)               return safety_margin_;
        const double v = cur_vs;
        if (v <= sm_v_lo_) return safety_margin_;
        if (v >= sm_v_hi_) return safety_margin_hi_;
        return safety_margin_ + (safety_margin_hi_ - safety_margin_)
                              * (v - sm_v_lo_) / (sm_v_hi_ - sm_v_lo_);
    }

    auto runOnline(const f110_msgs::msg::ObstacleArray &obstacles) -> pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray> {
        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray mrks;

        // 회피 대상 obstacle 필터링
        auto obs = obs_filtering(obstacles);
        
        // 회피 대상 obstacle 없는 경우 Online 마무리
        if (obs.empty()) {
            fail_ = FailSnapshot{}; fail_.reason = FailSnapshot::NO_OBS;
            fail_.n_obs = obstacles.obstacles.size();
            return {wpnts, mrks};
        }
    
        // obs가 존재하는 영역에 대하여 노드/엣지 차단
        f110_msgs::msg::Obstacle target_obstacle;  // 가장 가까운 앞쪽 장애물
        double min_gap = 1e9;

        std::vector<ObsZone> zones;
        std::set<int> touched_layers;
        for (auto &target_obs : obs) {
            double gap = target_obs.s_center - cur_s;

            // ego 앞쪽에 있는 장애물만 고려
            if (gap > 0.0 && gap < min_gap) {
                min_gap = gap;
                target_obstacle = target_obs;
            }

            ObsZone z = find_obs_jone(target_obs);
            if (z.layers.empty()) continue;
            touched_layers.insert(z.layers.begin(), z.layers.end());
            zones.push_back(std::move(z));
        }
        if (zones.empty()) {
            fail_ = FailSnapshot{}; fail_.reason = FailSnapshot::NO_ZONE; fail_.n_obs = obs.size();
            return {wpnts, mrks};
        }

        // ---- layer 별 적응형 여유(margin) ---------------------------------------
        // 평상시 여유(m_nom)로 시작하되, 그 layer 에 통과 가능한 노드가 하나도 안 남으면
        // 물리 하한(m_min)까지 lat_resolution 단위로 단계적으로 깎는다.
        // 좁은 구간에서만 여유를 줄이고 넓은 구간은 원래 여유를 그대로 유지하기 위해
        // 전역이 아니라 layer 단위로 완화한다.
        // safety_margin 은 속도에 따라 커진다(effective_safety_margin 참조).
        const double sm_eff = effective_safety_margin();
        const double m_nom = params.veh_width / 2.0 + sm_eff
                             + inflate_idx_ * params.lat_resolution;
        const double m_min = std::min(m_nom, params.veh_width / 2.0 + min_safety_margin_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "safety_margin: %.3f (v=%.2f m/s, base %.3f -> hi %.3f)",
            sm_eff, cur_vs, safety_margin_, safety_margin_hi_);

        std::set<IPair> blocked_nodes;
        for (int layer : touched_layers) {
            const int n_node = static_cast<int>(nodeMap[layer].size());
            std::set<int> chosen;
            double used = m_nom;

            for (double m = m_nom; ; m -= params.lat_resolution) {
                if (m < m_min) m = m_min;
                chosen = blocked_idx_at_layer(zones, layer, m);
                used = m;
                if (static_cast<int>(chosen.size()) < n_node) break;   // 자유 노드 확보
                if (m <= m_min + 1e-9) break;                          // 하한까지 완화했는데도 실패
            }

            if (static_cast<int>(chosen.size()) >= n_node) {
                // 물리 하한까지 완화해도 못 지나간다 = 실제로 폭이 부족한 구간
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "layer %d: 최소 여유(%.2f m)로도 통과 폭 없음 (%d/%d 노드 차단) -> 실제 회피 불가 구간",
                    layer, m_min, static_cast<int>(chosen.size()), n_node);
            } else if (used < m_nom - 1e-9) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "layer %d: 폭이 좁아 여유를 %.2f m -> %.2f m 로 완화 (자유 노드 %d개)",
                    layer, m_nom, used, n_node - static_cast<int>(chosen.size()));
            }

            for (int i : chosen) blocked_nodes.insert({layer, i});
        }

        // apply_node_filter 용 (layer, idx, idx) 리스트로 변환
        std::vector<std::tuple<int,int,int>> blocked_zones;
        blocked_zones.reserve(blocked_nodes.size());
        for (const auto &n : blocked_nodes) blocked_zones.emplace_back(n.first, n.second, n.second);

        // ---- 회피 방향 잠금 (2026-08-21) ------------------------------------
        // 잠그는 것은 '경로'가 아니라 '어느 쪽으로 피할지' 하나다. 경로 기하는 매 주기
        // 새로 풀되, 대상 장애물의 layer 에서 반대쪽 노드를 후보에서 빼서 방향만 유지한다.
        const bool   have_target = (min_gap < 1e8);
        const double target_s    = have_target ? target_obstacle.s_center : 0.0;

        // 잠금 유지/해제 판정.
        //   ★ 탐지가 끊겨 '더 먼' 장애물이 대상이 되는 일이 잦다(실측: 전방 6 m 장애물이
        //     프레임마다 있다 없다 한다). 그때 잠금을 풀면 잠금이 사실상 무의미해지므로,
        //     대상이 더 멀어진 경우는 탐지 끊김으로 보고 잠금을 유지한다.
        //   해제는 세 가지뿐이다: (a) 잠근 장애물을 지났다 (b) 더 '가까운' 장애물이
        //   새로 나타났다(= 새 정보) (c) 그 방향으로 연속 실패(아래 graph_search 뒤).
        const int prev_side = side_lock_ ? locked_side_ : 0;
        bool released_passed = false;
        if (!side_lock_) {
            locked_side_ = 0; lock_fail_ = 0;
        } else if (locked_side_ != 0) {
            const double lock_gap = lock_obs_s_ - cur_s;
            if (lock_gap <= 0.0) {
                RCLCPP_INFO(this->get_logger(),
                    "[side_lock] 해제: 장애물 통과 완료 (s %.2f)", lock_obs_s_);
                locked_side_ = 0; lock_fail_ = 0; released_passed = true;
            } else if (have_target && target_s < lock_obs_s_ - side_lock_match_dist_) {
                RCLCPP_INFO(this->get_logger(),
                    "[side_lock] 해제: 더 가까운 장애물 등장 (s %.2f -> %.2f)",
                    lock_obs_s_, target_s);
                locked_side_ = 0; lock_fail_ = 0;
            }
        }

        // 잠근 장애물의 zone 을 찾는다(이번 프레임에 안 보이면 nullptr).
        const ObsZone *lock_zone = nullptr;
        if (locked_side_ != 0 && !zones.empty()) {
            double best = 1e9;
            for (const auto &z : zones) {
                const double c = std::fabs(0.5 * (z.s_lo + z.s_hi) - lock_obs_s_);
                if (c < best) { best = c; lock_zone = &z; }
            }
            if (best > side_lock_match_dist_) lock_zone = nullptr;
            else lock_obs_d_ = 0.5 * (lock_zone->d_lo + lock_zone->d_hi);  // d 갱신
        }
        // 아직 안 잠갔으면 이번 대상 장애물의 zone 을 방향 판정 기준으로 쓴다.
        const ObsZone *ref_zone = lock_zone;
        if (locked_side_ == 0 && have_target && !zones.empty()) {
            double best = 1e9;
            for (const auto &z : zones) {
                const double c = std::fabs(0.5 * (z.s_lo + z.s_hi) - target_s);
                if (c < best) { best = c; ref_zone = &z; }
            }
        }

        // 동결 거리는 '잠근' 장애물 기준으로 잰다. 현재 대상 기준으로 재면 탐지가 한 프레임
        // 끊겼을 때 동결이 풀려 코앞에서 방향이 뒤집힌다.
        const double freeze_gap = (locked_side_ != 0) ? (lock_obs_s_ - cur_s) : min_gap;
        const bool   frozen     = side_lock_ && freeze_gap > 0.0
                                             && freeze_gap < side_freeze_dist_;

        // 잠근 방향의 반대쪽 노드를 후보에서 제외한다(잠근 장애물의 layer 에서만).
        std::set<IPair> search_blocked;
        if (hard_block_nodes_) search_blocked = blocked_nodes;
        if (locked_side_ != 0 && lock_zone != nullptr) {
            const double od = lock_obs_d_;
            for (int layer : lock_zone->layers) {
                if (layer < 0 || layer >= static_cast<int>(nodeMap.size())) continue;
                const int n_node = static_cast<int>(nodeMap[layer].size());
                for (int i = 0; i < n_node; ++i) {
                    const double d = node_d(layer, i);
                    if ((locked_side_ > 0 && d < od) || (locked_side_ < 0 && d > od))
                        search_blocked.insert({layer, i});
                }
            }
        }
        const std::set<IPair> *search_set =
            (hard_block_nodes_ || locked_side_ != 0) ? &search_blocked : nullptr;

        // obs가 점유하고 있는 공간을 어떻게 막을지.
        //   hard_block_nodes_ = true  : graph_search 가 그 노드를 아예 확장하지 않는다 (기본)
        //   hard_block_nodes_ = false : 예전 방식. 나가는 edge 에 +1e6 벌점만 준다
        // 벌점 방식은 우회 곡률 비용이 1e6 을 넘으면 "뚫고 가는 해"가 최적이 되어 경로가
        // 통째로 폐기된다. 하드 프루닝은 그 비교 자체를 없앤다. 자세한 배경은 graph_search 주석.
        if (!hard_block_nodes_) nodeGraph.apply_node_filter(blocked_zones);
        // nodeGraph.apply_node_filter(blocked_zones, target_obstacle, nodeMap);
        // 회피 경로의 시작 노드/목표 노드 정의
        auto [startIdx, endIdx] = findDestination(blocked_zones);

        // 회피 방향 일관성 유지
        // dynamic parameter: hyst_time_ -> 바뀌고 해당 시간만큼은 회피 방향 일관성 유지
        // TODO! 지능적으로 회피 방향 유지할 수 있게 로직 추가해야 한다.
        // if ((this->now() - last_switch_time).seconds() < hyst_time_) {
        //     nodeGraph.hysteresisBias(last_side, startIdx.first, nodeIndicesOnRaceline, nodeMap, 5);
        // }

        // 실패 시 원인을 남기기 위한 스냅샷 채우기 (실패 경로에서만 호출된다)
        auto snap = [&](FailSnapshot::Reason r, IPair off) {
            fail_.reason      = r;
            fail_.zones       = zones;
            fail_.m_nom       = m_nom;
            fail_.m_min       = m_min;
            fail_.offending   = off;
            fail_.start_layer = startIdx.first;
            fail_.dest_layer  = endIdx.first;
            fail_.n_obs       = obs.size();
            fail_.obs_gap     = (min_gap < 1e8) ? min_gap : -1.0;
            fail_.obs_d       = target_obstacle.d_center;
            fail_.obs_size    = target_obstacle.size;
            fail_.n_path      = 0;
        };

        // 최소 비용 경로 탐색. 하드 프루닝/방향 잠금일 때만 제외 노드 집합을 넘긴다.
        IPairVector nodeArray = nodeGraph.graph_search(
            startIdx, endIdx.first, nodeIndicesOnRaceline, this->get_logger(),
            true, search_set);

        // 잠근 방향으로 해가 없을 때.
        //   동결 구간 안  : 방향을 바꾸지 않는다. 빈 경로가 나가고 state machine 이
        //                   TRAILING 으로 감속한다(= "방향 대신 속도를 줄인다").
        //   동결 구간 밖  : 연속 side_unlock_fails_ 회 실패해야 잠금을 푼다. 한 프레임
        //                   흔들림으로 방향이 바뀌면 잠금이 무의미해지기 때문이다.
        if (nodeArray.size() < 2 && locked_side_ != 0) {
            if (frozen) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[side_lock] 동결(%.2f m < %.2f m): 잠근 방향(%s)에 해가 없다 "
                    "-> 방향 유지, 감속으로 대응",
                    min_gap, side_freeze_dist_, locked_side_ > 0 ? "left" : "right");
            } else if (++lock_fail_ >= side_unlock_fails_) {
                RCLCPP_INFO(this->get_logger(),
                    "[side_lock] 해제: %s 방향 %d회 연속 실패 -> 잠금 없이 재탐색",
                    locked_side_ > 0 ? "left" : "right", lock_fail_);
                locked_side_ = 0; lock_fail_ = 0;
                nodeArray = nodeGraph.graph_search(
                    startIdx, endIdx.first, nodeIndicesOnRaceline, this->get_logger(),
                    true, hard_block_nodes_ ? &blocked_nodes : nullptr);
            }
        }
        if (nodeArray.size() >= 2) lock_fail_ = 0;

        // ※ Edge COST 원상복구(deactivateFiltering)는 아래 스플라인 재탐색 루프가 끝난 뒤에
        //   한다. 여기서 먼저 풀면 hard_block_nodes:=false(벌점 방식)일 때 재탐색이
        //   장애물 벌점을 잃어버려 아무 제약 없이 다시 뚫는 해를 낸다.
    
        // RCLCPP_INFO(this->get_logger(), "ComputeSplines input: %zu nodes", nodeArray.size());
        if (nodeArray.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "graph_search 실패 (경로 노드 %zu개) -> 회피 경로 없음", nodeArray.size());
            snap(FailSnapshot::SEARCH_FAIL, {-1,-1});
            return {wpnts, mrks};
        }

        // 차단 영역을 통과하는 해가 나왔다면 그것은 "장애물을 뚫고 가는 경로"다.
        // 벌점 방식(hard_block_nodes_=false)에서는 penalty 가 유한하기 때문에 우회가 더 비싸지면
        // Dijkstra 가 이런 해를 낸다. 그대로 publish 하면 state machine 이 OVERTAKE 로 넘어가
        // 장애물에 박으므로 여기서 버린다.
        // 하드 프루닝(기본)에서는 그런 해가 만들어질 수 없어 이 검사는 절대 걸리지 않는다.
        // 그래도 안전망으로 남겨두고, 걸리면 프루닝에 구멍이 있다는 뜻이므로 ERROR 로 알린다.
        // (index 0 = 시작 노드는 이미 차량이 서 있는 자리라 제외)
        for (size_t i = 1; i < nodeArray.size(); ++i) {
            if (blocked_nodes.count(nodeArray[i])) {
                if (hard_block_nodes_) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[BUG] 하드 프루닝인데 경로가 차단 노드(layer %d, idx %d)를 지난다",
                        nodeArray[i].first, nodeArray[i].second);
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "최적 경로가 차단 영역(layer %d, idx %d)을 통과 -> 회피 불가로 판단, 빈 경로 publish",
                    nodeArray[i].first, nodeArray[i].second);
                snap(FailSnapshot::PATH_BLOCKED, nodeArray[i]);
                return {wpnts, mrks};
            }
        }

        // 경로 탐색 시 endIdx가 raceline 위가 아닌 경우 강제 맞춤(쿠션용)
        // auto [check_layer, check_idx] = nodeArray.back();
        // if (!nodeIndicesOnRaceline[check_layer] == check_idx) {
        //     RCLCPP_INFO(this->get_logger(), "Last Node isn't on Raceline!!!");
        //     int rl_idx = nodeIndicesOnRaceline[check_layer]; // 강제로 가장 가까운 raceline 노드로 수정
        //     nodeArray.back() = {check_layer, rl_idx};
        // }

        // ===== 노드 시퀀스 -> 스플라인 -> 충돌 검사 -> (필요하면) 재탐색 =====
        // 위 PATH_BLOCKED 검사는 '노드' 만 본다. 실제로 발행되는 것은 노드를 이은 곡선
        // 위의 샘플점이므로, 여기서 그 곡선을 장애물 zone 과 직접 대조한다.
        // zone 은 이미 연속 실수 구간(z.s_lo/s_hi, z.d_lo/d_hi)이라 노드 격자로 반올림하는
        // 손실이 없다. 걸리면 그 구간 하류 노드를 후보에서 빼고 다시 푼다 —
        // '경로 1개를 뽑고 버리는' 것이 아니라 '나쁜 선택지를 빼고 다시 고르는' 것이라,
        // 통과 가능한 공간이 있으면 대안이 나온다.
        const double track_length = stMap[RL_S].back();

        // b 에서 a 까지의 전방 거리 [0, L)
        auto s_fwd = [&](double a, double b) {
            double d = std::fmod(a - b, track_length);
            if (d < 0.0) d += track_length;
            return d;
        };

        // 탐색에서 제외할 노드 집합. 원래 차단(하드 프루닝/방향 잠금) 그대로이며,
        // 스플라인 검사는 여기에 손대지 않는다(엣지 벌점으로 처리한다).
        std::set<IPair> merged_blocked;
        if (search_set) merged_blocked = *search_set;

        std::vector<SplineSample> evasion_points;
        std::vector<double> s_vec, d_all;
        Eigen::VectorXd kappa, el_lengths;
        int N = 0;
        bool  path_ok = false;
        IPair offending{-1,-1};

        for (int attempt = 0; ; ++attempt) {
            MatrixXd path(nodeArray.size(), 2);
            for (size_t i = 0; i < nodeArray.size(); ++i) {
                auto [layer, idx] = nodeArray[i];
                const ::Node &n = nodeMap[layer][idx];
                path(i, 0) = n.x;
                path(i, 1) = n.y;
            }
            const double psi_s = nodeMap[nodeArray.front().first][nodeArray.front().second].psi;
            const double psi_e = nodeMap[nodeArray.back().first][nodeArray.back().second].psi;

            auto evasion_spline = nodeGraph.computeSplines(path, psi_s, psi_e, true);
            if (evasion_spline->coeffs_x.size() == 0 || evasion_spline->coeffs_y.size() == 0) {
                RCLCPP_WARN(this->get_logger(),
                            "Skipping waypoint generation: empty spline coefficients (path too short)");
                snap(FailSnapshot::SPLINE_EMPTY, {-1,-1});
                return {wpnts, mrks};
            }

            evasion_points = nodeGraph.interpSpline(evasion_spline->coeffs_x,
                                                    evasion_spline->coeffs_y);
            N = (int)evasion_points.size();
            if (N < 2) {
                snap(FailSnapshot::SPLINE_EMPTY, {-1,-1});
                return {wpnts, mrks};
            }

            kappa.resize(N);
            el_lengths.resize(N - 1);
            kappa(0) = evasion_points[0].kappa;
            for (int i = 1; i < N; ++i) {
                kappa(i) = evasion_points[i].kappa;
                const double dx = evasion_points[i].x - evasion_points[i-1].x;
                const double dy = evasion_points[i].y - evasion_points[i-1].y;
                el_lengths(i-1) = std::hypot(dx, dy);
            }

            s_vec.assign(N, 0.0);
            s_vec[0] = stMap[RL_S][nodeArray.front().first];   // 현재 위치 raceline s에서 시작
            for (int i = 1; i < N; ++i) {
                s_vec[i] = s_vec[i-1] + el_lengths(i-1);
                if (s_vec[i] >= track_length) s_vec[i] -= track_length;  // wrap-around
            }

            // 배치 frenet 변환. 예전에는 아래 발행 루프에서 점마다 1개짜리 벡터로 N 번
            // 호출했다(N 중앙 88). 한 번에 넘기는 쪽이 더 싸고, 그 결과를 검사와 발행이
            // 함께 쓰므로 이 검사를 넣고도 총 계산량은 늘지 않는다.
            {
                std::vector<double> xs(N), ys(N);
                for (int i = 0; i < N; ++i) { xs[i] = evasion_points[i].x; ys[i] = evasion_points[i].y; }
                auto fr = converter->get_frenet(xs, ys, &s_vec);
                d_all = fr.second;
            }

            if (!spline_check_ || zones.empty()) { path_ok = true; break; }

            // ---- 발행될 곡선을 장애물 zone(연속 구간)과 대조 ----
            // 여유는 m_min(= veh_width/2 + min_safety_margin) 을 쓴다. m_nom 을 쓰면
            // 좁은 구간에서 layer 별로 정당하게 완화된 경로까지 전부 버리게 된다.
            const double skip_m = (spline_check_skip_m_ > 0.0) ? spline_check_skip_m_
                                                               : params.veh_length * 0.5;
            int bad = -1;
            for (int i = 0; i < N && bad < 0; ++i) {
                // 차 바로 앞(반차장)은 면제. 차가 이미 장애물 옆에 붙어 있으면 이 구간은
                // 늘 걸리고, 그러면 어떤 경로도 못 낸다. 예전 'index 0 면제'가 노드 한 칸
                // (= 최대 0.80 m)이었던 것을 거리 기준(0.27 m)으로 좁힌 것이다.
                if (s_fwd(s_vec[i], cur_s) < skip_m) continue;
                for (const auto &z : zones) {
                    if (s_fwd(s_vec[i], z.s_lo) > s_fwd(z.s_hi, z.s_lo)) continue;  // s 범위 밖
                    if (d_all[i] >= z.d_lo - m_min && d_all[i] <= z.d_hi + m_min) { bad = i; break; }
                }
            }
            if (bad < 0) { path_ok = true; break; }

            if (attempt >= spline_check_retries_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "스플라인이 장애물을 지난다 (전방 %.2f m, s=%.2f d=%.2f). 재탐색 %d회 모두 실패 -> 빈 경로",
                    s_fwd(s_vec[bad], cur_s), s_vec[bad], d_all[bad], spline_check_retries_);
                break;
            }

            // ---- 충돌한 '엣지' 에 벌점을 주고 다시 푼다 ----------------------
            // ★ 2026-08-22 수정: 예전에는 도착 '노드' 를 후보에서 뺐다. 그건 틀렸다 —
            //   충돌은 노드가 아니라 두 노드를 잇는 곡선(엣지)의 성질이고, 그 노드 자체는
            //   다른 앞 노드에서 오면 멀쩡할 수 있다. 노드를 빼면 멀쩡한 선택지가 사라지고,
            //   장애물이 이미 그 레이어를 대부분 막아 자유 노드가 1~2개뿐일 때는
            //   생존자를 지워 통로를 스스로 끊는다.
            //   실측(obs_debug_0822_2017 재생): 노드 제외 방식에서 재탐색이
            //   "경로 노드 0개" 로 죽은 것이 14건, 3회를 다 쓰고 실패한 것이 42건이었다.
            //   엣지 벌점은 apply_node_filter 가 쓰는 기계(splineMap[src][dst].cost +
            //   orig_edges 복원)를 그대로 재사용하므로 추가 자료구조가 없다.
            //   복원은 루프 뒤의 deactivateFiltering() 이 한다.
            size_t k = 1;
            for (; k < nodeArray.size(); ++k) {
                if (s_fwd(stMap[RL_S][nodeArray[k].first], s_vec[bad]) < track_length * 0.5) break;
            }
            if (k >= nodeArray.size()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "스플라인 충돌인데 벌점 줄 하류 엣지가 없다 -> 빈 경로 (s=%.2f)", s_vec[bad]);
                break;
            }
            const IPair e_src = nodeArray[k-1];
            const IPair e_dst = nodeArray[k];

            auto &sm = nodeGraph.getSplineMap();
            auto it_src = sm.find(e_src);
            if (it_src == sm.end() || it_src->second.find(e_dst) == it_src->second.end()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "스플라인 충돌 엣지를 splineMap 에서 못 찾음 -> 빈 경로");
                break;
            }
            {
                double &ec = it_src->second[e_dst].cost;
                // 원래 비용은 한 번만 기록한다(이미 apply_node_filter 가 건드렸을 수도 있다).
                if (nodeGraph.orig_edges.find({e_src, e_dst}) == nodeGraph.orig_edges.end())
                    nodeGraph.orig_edges[{e_src, e_dst}] = ec;
                // 벌점은 1e6 이 아니라 1e9 를 쓴다. 이 그래프의 정상 비용 상한은
                // w_virt_goal(1e4) + w_curv_avg(7e3) 수준이라 1e6 이면 충분해 보이지만,
                // 재탐색을 여러 번 하면 벌점이 쌓인 엣지끼리 비교가 되므로 여유를 크게 둔다.
                ec += 1e9;
            }
            offending = e_dst;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "스플라인이 장애물을 지나 엣지(L%d,%d)->(L%d,%d) 벌점 후 재탐색 (%d회차, 전방 %.2f m)",
                e_src.first, e_src.second, e_dst.first, e_dst.second,
                attempt + 1, s_fwd(s_vec[bad], cur_s));

            nodeArray = nodeGraph.graph_search(startIdx, endIdx.first, nodeIndicesOnRaceline,
                                               this->get_logger(), true, &merged_blocked);
            if (nodeArray.size() < 2) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "스플라인 충돌 회피 재탐색 실패 (경로 노드 %zu개) -> 빈 경로", nodeArray.size());
                break;
            }
        }

        // Edge COST 원상복구 (하드 프루닝이면 손댄 edge 가 없어 no-op)
        nodeGraph.deactivateFiltering();

        if (!path_ok) {
            snap(FailSnapshot::PATH_BLOCKED, offending);
            return {wpnts, mrks};
        }

        auto [cur_layer, cur_idx] = nodeArray.front();

        // 속도 프로파일 생성
        auto [end_layer, end_idx] = nodeArray.back();
        double v_start = stMap[RL_VX][cur_layer];
        // double v_start = cur_vs;
        double v_end   = stMap[RL_VX][end_layer]; 

        // VpForwardBackward vp(3.0, 6.0, params.vel_max, params.gg_scale);
        // vp.updateDynParameters(params.vel_max, params.gg_scale);

        // Eigen::VectorXd vx = vp.calcVelProfile(
        //     kappa, el_lengths,
        //     v_start, v_end,
        //     target_obstacle,
        //     overtake_speed_gain_     
        // );
        // Eigen::VectorXd ax = accelFromProfile(vx, el_lengths);
        VpForwardBackward vp(params);


        // 속도 프로파일 계산
        Eigen::VectorXd vx = vp.calcVelProfile(kappa, el_lengths, v_start, v_end);


        // 가속도 계산 (간단한 finite difference)
        Eigen::VectorXd ax(vx.size());
        for (int i = 0; i < vx.size() - 1; ++i) {
            double ds = std::max(1e-6, el_lengths(i));
            ax(i) = (vx(i + 1) * vx(i + 1) - vx(i) * vx(i)) / (2.0 * ds);
        }
        ax(vx.size() - 1) = ax(vx.size() - 2);



        // fill wpnts
        for (int i=0; i<N; ++i) {
            // frenet 은 위에서 배치로 한 번에 구해 뒀다 (검사와 공유).
            double psi = std::atan2(evasion_points[i].y_d, evasion_points[i].x_d);
            auto w = xypsi_to_wpnt(evasion_points[i].x, evasion_points[i].y,
                                s_vec[i], d_all[i],
                                psi, kappa(i),
                                vx(i), ax(i), i);
            wpnts.wpnts.push_back(w);

            visualization_msgs::msg::Marker m = xyv_to_marker(evasion_points[i].x, evasion_points[i].y, vx(i), i);
            mrks.markers.push_back(m);

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

        // ---- 이번 주기에 실제로 고른 방향을 기록하고 메시지에 싣는다 -----------
        // ot_side / last_switch_time 은 state machine 이 읽는다(splini_hyst_timer_sec).
        // 2026-08-21 이전에는 이 두 필드를 아무도 채우지 않아 그 히스테리시스가 죽어 있었다.
        if (!wpnts.wpnts.empty() && (locked_side_ != 0 || (have_target && ref_zone != nullptr))) {
            const double ref_s = (locked_side_ != 0) ? lock_obs_s_ : target_s;
            const double ref_d = (locked_side_ != 0) ? lock_obs_d_
                                                     : 0.5 * (ref_zone->d_lo + ref_zone->d_hi);
            double best = 1e9, d_at_obs = 0.0;
            for (const auto &w : wpnts.wpnts) {
                const double c = std::fabs(w.s_m - ref_s);
                if (c < best) { best = c; d_at_obs = w.d_m; }
            }
            const int side_now = (d_at_obs > ref_d) ? +1 : -1;

            // 방향이 실제로 뒤집혔을 때만 시각을 찍는다. state machine 이 그 뒤
            // splini_hyst_timer_sec 동안 이 경로를 쓰지 않는다. 장애물을 지나 새 장애물로
            // 넘어가며 방향이 달라진 것은 flip-flop 이 아니므로 찍지 않는다
            // (그것까지 막으면 새 장애물에 대한 회피 시작이 늦어진다).
            if (locked_side_ == 0) {
                locked_side_ = side_now;
                lock_obs_s_  = ref_s;
                lock_obs_d_  = ref_d;
                if (prev_side != 0 && !released_passed && side_now != prev_side) {
                    last_switch_time  = this->now();
                    wpnts.side_switch = true;
                    RCLCPP_WARN(this->get_logger(),
                        "[side_lock] 방향 뒤집힘 %s -> %s (장애물 %.2f m 앞)",
                        prev_side > 0 ? "left" : "right",
                        side_now > 0 ? "left" : "right", ref_s - cur_s);
                }
            } else if (side_now != locked_side_) {
                // 잠겨 있는데도 반대쪽으로 나왔다 = 이번 프레임에 그 장애물이 안 보여
                // 제약을 못 걸었다는 뜻이다. 사실대로 기록한다.
                RCLCPP_WARN(this->get_logger(),
                    "[side_lock] 잠금(%s)과 다른 방향(%s)이 나왔다 (장애물 미검출 프레임)",
                    locked_side_ > 0 ? "left" : "right", side_now > 0 ? "left" : "right");
                last_switch_time  = this->now();
                wpnts.side_switch = true;
                locked_side_      = side_now;
            }
            wpnts.ot_side = (locked_side_ > 0) ? "left" : "right";
        }
        wpnts.last_switch_time = last_switch_time;

        fail_ = FailSnapshot{};
        fail_.reason = FailSnapshot::OK;
        fail_.n_path = wpnts.wpnts.size();
        return std::make_pair(wpnts, mrks);
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
    nodeMap.resize(N);
    nodeIndicesOnRaceline.clear();

    for (int i = 0; i < N; ++i) {
        ::Node node_;
        int raceline_index = floor((stMap[WIDTH_L][i] + stMap[NORM_L][i] - params.veh_width / 2) / params.lat_resolution);
        nodeIndicesOnRaceline.push_back(raceline_index);

        Vector2d ref_xy(stMap[POS_X][i], stMap[POS_Y][i]);
        Vector2d norm_vec(stMap[NORM_X][i], stMap[NORM_Y][i]);

        // === 노드 수 계산 ===
        int num_nodes = (stMap[WIDTH_R][i] + stMap[WIDTH_L][i] - params.veh_width) / params.lat_resolution;
        if (num_nodes == raceline_index) num_nodes++;
        nodeMap[i].resize(num_nodes);

        // === 좌우 PSI 보간 방향 보정 ===
        double psi_LB = normalizeAngle(stMap[LB_PSI][i]);
        double psi_RL = normalizeAngle(stMap[RL_PSI][i]);
        double psi_RB = normalizeAngle(stMap[RB_PSI][i]);

        // wrap-around 방지: ψ 차이가 180도 이상이면 보정
        auto psi_diff = [](double a, double b){
            double d = b - a;
            while (d > M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            return d;
        };

        double diff_L_to_R = psi_diff(psi_LB, psi_RL);
        double diff_R_to_B = psi_diff(psi_RL, psi_RB);

        for (int idx = 0; idx < num_nodes; ++idx) {
            double alpha = stMap[NORM_L][i] - raceline_index * params.lat_resolution + idx * params.lat_resolution;
            Vector2d node_pos = ref_xy + alpha * norm_vec;

            node_.x = node_pos.x();
            node_.y = node_pos.y();
            node_.raceline = (idx == raceline_index);

            double psi_interp = 0.0;

            if (idx < raceline_index) {
                // 왼쪽 구간: LB → RL로 보간
                double t = static_cast<double>(idx) / std::max(raceline_index, 1);
                psi_interp = psi_LB + diff_L_to_R * t;
            }
            else if (idx == raceline_index) {
                // 레이싱라인: 그대로 사용
                psi_interp = psi_RL;
            }
            else {
                // 오른쪽 구간: RL → RB로 보간
                int remain = num_nodes - raceline_index - 1;
                double t = static_cast<double>(idx - raceline_index) / std::max(remain, 1);
                psi_interp = psi_RL + diff_R_to_B * t;
            }

            node_.psi = normalizeAngle(psi_interp);
            // node_.kappa = stMap[RL_KAPPA][i];  // 필요시 곡률도 raceline 기준으로
            nodeMap[i][idx] = node_;
        }
    }

    RCLCPP_INFO(this->get_logger(), "createNodeMap: total %zu layers built", nodeMap.size());
    return {nodeMap, nodeIndicesOnRaceline};
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

    bool is_same_path(const f110_msgs::msg::OTWpntArray& a,
                  const f110_msgs::msg::OTWpntArray& b,
                  double tol)
    {
        if (a.wpnts.size() != b.wpnts.size()) return false;
        for (size_t i=0; i<a.wpnts.size(); i++) {
            if (fabs(a.wpnts[i].x_m - b.wpnts[i].x_m) > tol) return false;
            if (fabs(a.wpnts[i].y_m - b.wpnts[i].y_m) > tol) return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Online Loop /////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    // ======================= 정지 원인 진단 =======================
    // "차가 멈춰 있는데 왜 안 가느냐" 를 로그 한 블록으로 답한다.
    // 호출 조건이 (정지 지속 + 장애물 보임 + 주기 경과) 라서 정상 주행 중에는 돌지 않는다.
    // 비용: layer 12개 x 노드 ~18개 x 자식 ~17개 = 수천 회 map 조회, 2 초에 1 회.

    // 연속된 자유 인덱스 구간을 d 범위로 찍는다. (인덱스 증가 = d 감소일 수 있어 min/max 로)
    void append_free_intervals(std::ostringstream &os, int layer, const std::set<int> &freeset) {
        if (freeset.empty()) { os << " (없음)"; return; }
        int run_start = -1, prev = -1000;
        auto flush = [&](int a, int b) {
            const double d1 = node_d(layer, a), d2 = node_d(layer, b);
            os << " d[" << std::showpos << std::min(d1,d2) << ".." << std::max(d1,d2) << "]" << std::noshowpos;
        };
        for (int i : freeset) {
            if (i != prev + 1) { if (run_start >= 0) flush(run_start, prev); run_start = i; }
            prev = i;
        }
        if (run_start >= 0) flush(run_start, prev);
    }

    // runOnline 과 동일한 완화 규칙으로 그 layer 의 자유 노드를 재현한다.
    std::set<int> free_nodes_at(int layer) const {
        std::set<int> freeset, chosen;
        const int n = static_cast<int>(nodeMap[layer].size());
        for (double m = fail_.m_nom; ; m -= params.lat_resolution) {
            if (m < fail_.m_min) m = fail_.m_min;
            chosen = blocked_idx_at_layer(fail_.zones, layer, m);
            if (static_cast<int>(chosen.size()) < n) break;
            if (m <= fail_.m_min + 1e-9) break;
        }
        for (int i = 0; i < n; ++i) if (!chosen.count(i)) freeset.insert(i);
        return freeset;
    }

    void append_corridor_report(std::ostringstream &os) {
        if (fail_.zones.empty() || nodeMap.empty()) return;
        const int nL = static_cast<int>(nodeMap.size());

        // 1) 차단된 layer 들의 자유 노드 현황
        std::set<int> touched;
        for (const auto &z : fail_.zones) {
            touched.insert(z.layers.begin(), z.layers.end());
            // 꼬리 layer 는 차단 대상이 아니어도 현황은 같이 보여준다 (통로 판단에 필요).
            if (z.tail_layer >= 0) touched.insert(z.tail_layer);
        }
        os << "  여유 margin: nom " << fail_.m_nom << " m / min " << fail_.m_min << " m\n";
        for (int l : touched) {
            if (l < 0 || l >= nL) continue;
            const auto fr = free_nodes_at(l);
            os << "  layer " << l << " s=" << stMap.at(RL_S)[l]
               << " 자유 " << fr.size() << "/" << nodeMap[l].size();
            append_free_intervals(os, l, fr);
            // 이 layer 가 장애물 뒤 "꼬리 layer" 인지 = 장애물의 실제 s 범위 밖인지
            for (const auto &z : fail_.zones) {
                if (z.tail_layer == l) {
                    os << "  <- 장애물 s[" << z.s_lo << ".." << z.s_hi << "] 밖의 꼬리 layer ("
                       << (block_obs_tail_layer_ ? "차단함" : "차단 안 함") << ")";
                    break;
                }
            }
            os << "\n";
        }

        // 2) 자유 노드만 밟고 목적지까지 갈 수 있는지 앞으로 굴려본다.
        auto xy = converter->get_cartesian(cur_s, cur_d);
        IPair startIdx = getClosestNodes(Eigen::Vector2d(xy.first, xy.second), 1);
        std::set<int> reach = free_nodes_at(startIdx.first);
        reach.insert(startIdx.second);   // 차가 이미 서 있는 자리는 통과시킨다
        int prev_layer = startIdx.first;
        for (int k = 1; k <= min_plan_horizon_; ++k) {
            const int cur_layer = (startIdx.first + k) % nL;
            const auto fr = free_nodes_at(cur_layer);
            std::set<int> next;
            for (int i : reach)
                for (const auto &c : nodeGraph.getChildList({prev_layer, i}))
                    if (c.first == cur_layer && fr.count(c.second)) next.insert(c.second);
            if (next.empty()) {
                os << "  >> 통로 단절: layer " << prev_layer << " -> layer " << cur_layer
                   << " 로 이어지는 자유 노드가 0개.\n"
                   << "     양쪽 모두 자유 폭은 남아 있으나 좌우로 갈라져 연결되지 않는다"
                      " (= 폭 부족이 아니다).\n";
                return;
            }
            reach = std::move(next);
            prev_layer = cur_layer;
        }
        if (hard_block_nodes_) {
            os << "  >> 장애물 기준으로는 자유 통로가 목적 layer 까지 이어져 있다.\n"
                  "     그런데도 graph_search 가 실패했다면 오프라인 그래프 자체가 끊긴 것이다"
                  " (pruneEdges 의 prune_kappa_max 로 엣지가 삭제됐는지 확인).\n";
        } else {
            os << "  >> 자유 통로는 목적 layer 까지 이어져 있다. 그런데도 차단 노드를 지나는 해가"
                  " 나왔다면\n     우회 경로의 곡률 비용이 차단 벌점(1e6)보다 비싼 경우다"
                  " (w_curv_avg/w_curv_peak 확인).\n";
        }
    }

    void report_stall(double stalled_sec) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(2);
        os << "\n[정지진단] " << std::setprecision(1) << stalled_sec << " s 정지"
           << (fail_.reason == FailSnapshot::OK ? ", 회피경로 있음\n" : ", 회피경로 없음\n")
           << std::setprecision(2);
        os << "  ego s=" << cur_s << " d=" << cur_d << " v=" << cur_vs
           << " | 장애물 " << fail_.n_obs << "개";
        if (fail_.obs_gap >= 0.0)
            os << ", 최근접 전방 " << fail_.obs_gap << " m (d=" << std::showpos << fail_.obs_d
               << std::noshowpos << " size=" << fail_.obs_size << ")";
        os << "\n  원인: ";
        switch (fail_.reason) {
            case FailSnapshot::OK:
                os << "플래너는 경로를 냈다(" << fail_.n_path << "점)."
                      " 정지 원인은 플래너 밖(state machine / 컨트롤러)이다.\n";
                break;
            case FailSnapshot::NO_OBS:
                os << "회피 대상 장애물 없음. |d| > obs_traj_tresh(" << obs_traj_tresh_
                   << " m) 이거나 전방 obs_lookahead(" << obs_lookahead_ << " m) 밖이다.\n";
                break;
            case FailSnapshot::NO_ZONE:
                os << "장애물이 어느 layer 에도 매핑되지 않음 (find_obs_jone 결과 비어 있음).\n";
                break;
            case FailSnapshot::SEARCH_FAIL:
                os << "graph_search 가 경로를 못 찾음 (start layer " << fail_.start_layer
                   << " -> dest layer " << fail_.dest_layer << ").\n";
                break;
            case FailSnapshot::SPLINE_EMPTY:
                os << "스플라인 계수가 비어 경로 생성 실패 (경로가 너무 짧다).\n";
                break;
            case FailSnapshot::PATH_BLOCKED:
                os << "최적 경로가 차단 노드(layer " << fail_.offending.first
                   << ", idx " << fail_.offending.second << ")를 지나 통째로 폐기됨.\n";
                break;
        }
        if (fail_.reason == FailSnapshot::PATH_BLOCKED || fail_.reason == FailSnapshot::SEARCH_FAIL)
            append_corridor_report(os);
        RCLCPP_WARN(this->get_logger(), "%s", os.str().c_str());
    }

    // online_loop 끝에서 매 주기 호출된다. 조건을 안 만족하면 즉시 return 하므로
    // 평상시 비용은 비교 몇 번이 전부다.
    void update_stall_diag() {
        if (!stall_diag_) return;
        // 장애물이 안 보이는데 서 있는 건(대기/수동정지) 플래너가 답할 문제가 아니다.
        if (std::fabs(cur_vs) > stall_speed_thresh_ || obs_msg.obstacles.empty()) {
            stall_since_ = rclcpp::Time(0,0,RCL_ROS_TIME);
            return;
        }
        const rclcpp::Time now = this->now();
        if (stall_since_.nanoseconds() == 0) { stall_since_ = now; return; }
        const double stalled = (now - stall_since_).seconds();
        if (stalled < stall_report_after_) return;
        if (last_stall_report_.nanoseconds() != 0 &&
            (now - last_stall_report_).seconds() < stall_report_period_) return;
        last_stall_report_ = now;
        report_stall(stalled);
    }

    void online_loop() {
        auto start_time = std::chrono::high_resolution_clock::now();
        // double wall_start = get_wall_time();
        // double cpu_start  = get_cpu_time();

        f110_msgs::msg::OTWpntArray wpnts;
        visualization_msgs::msg::MarkerArray mrks;

        // 마커 지우개
        visualization_msgs::msg::Marker del;
        del.header.stamp = this->now();
        del.header.frame_id = "map";   // frame도 반드시 지정
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        visualization_msgs::msg::MarkerArray clear;
        clear.markers.push_back(del);
        mrks_pub->publish(clear);
    
        // obs 있는 경우에만 online part 수행 
        if (!obs_msg.obstacles.empty()) {
            std::tie(wpnts, mrks) = runOnline(obs_msg);
            // (2026-08-21) 여기서 매 주기 last_switch_time = now() 로 덮어쓰고 있었다.
            // 그러면 "방향이 바뀐 시각"이라는 의미가 사라진다. 이제 runOnline 안에서
            // 실제로 방향이 뒤집혔을 때만 갱신하고, 메시지에도 거기서 싣는다.
            last_wpnts = wpnts; 
        } 

        wpnts.header.stamp = this->now();
        wpnts.header.frame_id = "map";
        evasion_pub->publish(wpnts);

        // 멈춰 있으면 왜 못 가는지 로그에 남긴다 (조건 불충족 시 즉시 return)
        update_stall_diag();

        // ---- 레이어 스냅 진단 리포트 (5초마다) ----
        if (snap_diag_ && snap_n_ > 0) {
            const auto now = this->now();
            if (snap_last_report_.nanoseconds() == 0) snap_last_report_ = now;
            if ((now - snap_last_report_).seconds() >= 5.0) {
                RCLCPP_WARN(this->get_logger(),
                    "\n===== 레이어 스냅 진단 (s기반 vs xy최근접) =====\n"
                    "  시작노드: 비교 %ld회, 레이어 불일치 %ld회 (%.1f%%), 평균 차이 %.2f, 최대 차이 %d\n"
                    "  장애물존: 비교 %ld회, span 불일치 %ld회 (%.1f%%), 평균 span %.2f(xy) -> %.2f(s), 최대 span(xy) %ld\n"
                    "  현재 적용중인 방식: %s",
                    snap_n_, snap_mismatch_, 100.0 * snap_mismatch_ / std::max(1L, snap_n_),
                    (double)snap_gap_sum_ / std::max(1L, snap_n_), (int)snap_gap_max_,
                    zone_n_, zone_mismatch_, 100.0 * zone_mismatch_ / std::max(1L, zone_n_),
                    (double)zone_span_old_ / std::max(1L, zone_n_),
                    (double)zone_span_new_ / std::max(1L, zone_n_), zone_span_old_max_,
                    layer_lookup_by_s_ ? "s 기반" : "xy 최근접(예전)");
                snap_last_report_ = now;
            }
        }

        if (measuring) {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            std_msgs::msg::Float32 latency;
            latency.data = static_cast<float>(elapsed.count());
            latency_pub->publish(latency);
        }

        mrks_pub->publish(mrks);

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

    double front_dist = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

    if (front_dist < closest_obs_) {
        double delta_s = 0.0, delta_d = 0.0;
        double ot_distance = fmod((obs.s_center - cur_s + wpnt_max_s), wpnt_max_s);

        int idx = std::min<int>(
            std::max<int>(0, static_cast<int>(cur_s * 10)),
            static_cast<int>(ltpl_wpnts_msg.ltplwpnts.size()) - 1
        );

        // 상대 속도 (내 racetraj 속도 - 상대 속도)
        double rel_speed = cur_vs - obs.vs;

        // 상대가 더 빠른 경우 → 나는 추월당하는 상황
        if (rel_speed <= 0.0) {
            // 이미 멀어지고 있으니 예측 보정 안 함
            return obs;
        }

        // 내가 더 빠른 경우 → 추월하는 상황
        // 예측 시간 계산 (최대 5초까지)
        double ot_time_distance = std::clamp(ot_distance / std::max(rel_speed, 0.1), 0.0, 2.0) * 0.5;

        // 위치 보정
        // dynamic parameter: obs_delay_s
        // dynamic parameter: obs_delay_d
        delta_s = ot_time_distance * obs.vs + obs_delay_s_; // 뒤쪽으로 offset
        delta_d = ot_time_distance * obs.vd + obs_delay_d_; // 좌우로 보정

        // s 업데이트
        obs.s_start = fmod(obs.s_start + delta_s + wpnt_max_s, wpnt_max_s);
        obs.s_center = fmod(obs.s_center + delta_s + wpnt_max_s, wpnt_max_s);
        obs.s_end   = fmod(obs.s_end   + delta_s + wpnt_max_s, wpnt_max_s);

        // d 업데이트
        obs.d_left   += delta_d;
        obs.d_center += delta_d;
        obs.d_right  += delta_d;

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
        std::vector<double> s_vec = {obs.s_start, obs.s_start, obs.s_end, obs.s_end};
        std::vector<double> d_vec = {obs.d_left,  obs.d_right, obs.d_right, obs.d_left};
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

    }
    return obs;
}

    std::vector<f110_msgs::msg::Obstacle> obs_filtering(const f110_msgs::msg::ObstacleArray &obstacles) {
        std::vector<f110_msgs::msg::Obstacle> close_obs;
        if (wpnt_max_s <= 0.0) return close_obs;

        for (auto obs : obstacles.obstacles) {
            // dynamic parameter: obs_traj_tresh_ -> raceline 에서 이만큼 떨어진 장애물은 회피 대상 아님
            if (std::abs(obs.d_center) > obs_traj_tresh_) continue;

            obs = predict_obs_movement(obs);

            // dynamic parameter: obs_lookahead_ -> 전방 이 거리 안의 장애물만 그래프를 막는다.
            // 이 필터가 꺼져 있으면 뒤/멀리 있는 장애물까지 layer 를 막아버려서
            // 정작 지금 필요한 회피 구간에 자유 노드가 남지 않는다.
            const double gap = std::fmod(obs.s_center - cur_s + wpnt_max_s, wpnt_max_s);
            if (gap > obs_lookahead_) continue;

            close_obs.push_back(obs);
        }

        return close_obs;
    }
    
    // 장애물 하나가 점유하는 영역. 팽창(inflation)은 적용하지 않은 "코어" 상태로 돌려준다.
    // 팽창량은 runOnline 에서 layer 별로 정하기 때문이다(좁은 구간 완화).
    struct ObsZone {
        std::vector<int> layers;      // 차단 대상 layer 들 (= 장애물이 실제로 걸친 구간)
        int    tail_layer{-1};        // 그 바로 뒤 layer. 장애물 s 범위 밖이라 기본은 차단하지 않는다.
                                      // block_obs_tail_layer_ 가 true 일 때만 layers 에도 들어간다.
        double d_lo{0.0}, d_hi{0.0};  // 장애물의 frenet d 범위 [m] (팽창 전)
        double s_lo{0.0}, s_hi{0.0};  // 장애물의 실제 frenet s 범위 [m] (진단 출력용)
    };

    // ---- 정지 원인 진단용 스냅샷 (2026-08-20) -------------------------------
    // runOnline 이 빈 경로를 낼 때마다 "왜" 를 여기 적어 둔다. 기록 비용은
    // 스칼라 몇 개 + zones 복사(보통 1~3개)뿐이다. 실제 분석(레이어별 자유 노드,
    // 통로 연결성)은 report_stall() 안에서만, 그것도 주기당 1회만 돈다.
    struct FailSnapshot {
        enum Reason { OK, NO_OBS, NO_ZONE, SEARCH_FAIL, PATH_BLOCKED, SPLINE_EMPTY };
        Reason reason{OK};
        std::vector<ObsZone> zones;
        double m_nom{0.0}, m_min{0.0};
        IPair  offending{-1,-1};      // PATH_BLOCKED 일 때 경로가 지나간 차단 노드
        int    start_layer{-1}, dest_layer{-1};
        size_t n_obs{0};
        double obs_gap{-1.0}, obs_d{0.0}, obs_size{0.0};  // 최근접 전방 장애물
        size_t n_path{0};             // 성공했을 때 낸 웨이포인트 수
    };
    FailSnapshot fail_;

    // 노드 인덱스 -> raceline 기준 frenet d [m]
    double node_d(int layer, int idx) const {
        return d_sign_ * (idx - nodeIndicesOnRaceline[layer]) * params.lat_resolution;
    }

    // ---- s/d 기반 레이어·노드 조회 (2026-08-20) -----------------------------
    // getClosestNodes 는 (x,y) 유클리드 최근접으로 전 레이어를 뒤진다. 레이어의 가로 길이
    // (트랙 폭, 최대 2.2 m)가 레이어 간격(d_curve 0.8 m)보다 크기 때문에, 코너 안쪽처럼
    // 레이어들이 부챗살로 모이는 곳에서는 엉뚱한 레이어가 더 가까워질 수 있다.
    // s 는 트랙을 따라간 거리라서 접힘/수렴에 원리적으로 면역이다.

    // s [m] -> layer. stMap[RL_S] 는 단조 증가라 이분 탐색으로 충분하다.
    int layer_at_s(double s) const {
        const auto &S = stMap.at(RL_S);
        const int nL = static_cast<int>(S.size());
        if (nL == 0) return 0;
        const double L = (wpnt_max_s > 1e-6) ? wpnt_max_s : S.back();
        s = std::fmod(std::fmod(s, L) + L, L);
        auto it  = std::lower_bound(S.begin(), S.end(), s);
        int hi   = static_cast<int>(std::distance(S.begin(), it)) % nL;
        int lo   = (hi - 1 + nL) % nL;
        auto ds  = [&](int l){ double d = std::fabs(s - S[l]); return std::min(d, L - d); };
        return ds(lo) <= ds(hi) ? lo : hi;
    }

    // raceline 기준 frenet d [m] -> layer 안의 노드 인덱스 (node_d 의 역함수)
    int node_idx_at_d(int layer, double d) const {
        const int rl = nodeIndicesOnRaceline[layer];
        const int n  = static_cast<int>(nodeMap[layer].size());
        int idx = rl + static_cast<int>(std::lround(d / (d_sign_ * params.lat_resolution)));
        return std::max(0, std::min(idx, n - 1));
    }

    // 두 레이어 인덱스의 wrap 을 고려한 거리
    int layer_gap(int a, int b) const {
        const int nL = static_cast<int>(nodeMap.size());
        int d = std::abs(a - b);
        return std::min(d, nL - d);
    }

    // 장애물이 걸치는 layer 와, raceline 기준 상대 노드 인덱스 범위를 계산한다.
    // 장애물 자체의 폭은 네 모서리(d_left/d_right) 매핑에 이미 반영돼 있다.
    // TODO! Node 구조체가 (s, d) 를 들고 있으면 getClosestNodes 없이 바로 계산할 수 있다.
    ObsZone find_obs_jone(const f110_msgs::msg::Obstacle &target_obs) {
        ObsZone zone;

        const int nL = static_cast<int>(nodeMap.size());
        if (nL == 0 || static_cast<int>(nodeIndicesOnRaceline.size()) != nL) return zone;

        // --- 종방향: 장애물이 걸치는 layer 범위 ---
        // s 로 직접 구한다. 모서리는 원래 '레이어 범위'를 얻는 데만 쓰였고(횡방향 범위는
        // 아래 zone.d_lo/d_hi 가 따로 들고 있다), 모서리 4개의 min/max 를 쓰기 때문에
        // 하나만 엉뚱한 레이어로 스냅되면 그 사이 레이어가 전부 막혔다.
        // s 기반이면 span 이 정확하고, 시작선을 걸친 장애물(s_start > s_end, predict_obs_movement
        // 의 fmod 때문에 생긴다)도 (end - start + nL) % nL 이 그대로 전방 span 으로 처리한다.
        int start_layer, end_layer;
        {
            const int s_start_layer = layer_at_s(target_obs.s_start);
            const int s_end_layer   = layer_at_s(target_obs.s_end);

            if (layer_lookup_by_s_ && !snap_diag_) {
                start_layer = s_start_layer; end_layer = s_end_layer;
            } else {
                auto p1 = converter->get_cartesian(target_obs.s_start, target_obs.d_left);
                auto p2 = converter->get_cartesian(target_obs.s_start, target_obs.d_right);
                auto p3 = converter->get_cartesian(target_obs.s_end,   target_obs.d_left);
                auto p4 = converter->get_cartesian(target_obs.s_end,   target_obs.d_right);
                const IPair corner[4] = {
                    getClosestNodes(Eigen::Vector2d(p1.first, p1.second)),
                    getClosestNodes(Eigen::Vector2d(p2.first, p2.second)),
                    getClosestNodes(Eigen::Vector2d(p3.first, p3.second)),
                    getClosestNodes(Eigen::Vector2d(p4.first, p4.second))
                };
                int xs = corner[0].first, xe = corner[0].first;
                for (int i = 1; i < 4; ++i) {
                    xs = std::min(xs, corner[i].first);
                    xe = std::max(xe, corner[i].first);
                }
                if ((xe - xs) > nL / 2) std::swap(xs, xe);

                if (snap_diag_) {
                    const int span_old = (xe - xs + nL) % nL + 1;
                    const int span_new = (s_end_layer - s_start_layer + nL) % nL + 1;
                    zone_n_++; zone_span_old_ += span_old; zone_span_new_ += span_new;
                    if (span_old > zone_span_old_max_) zone_span_old_max_ = span_old;
                    if (span_old != span_new) zone_mismatch_++;
                }
                if (layer_lookup_by_s_) { start_layer = s_start_layer; end_layer = s_end_layer; }
                else                    { start_layer = xs;            end_layer = xe;          }
            }
        }

        // --- 횡방향: frenet d 범위를 그대로 들고 간다 ---
        // 예전엔 모서리를 최근접 노드로 반올림한 뒤 정수 팽창을 더했다. 양쪽 모두 바깥쪽으로
        // 반올림돼서 장애물 1개당 최대 0.1 m 를 그냥 버렸고, 폭 1.2 m 트랙에선 이게 치명적이다.
        // 이제 노드 d 를 직접 계산해 겹침을 판정하므로 반올림 손실이 없다.
        zone.d_lo = std::min(target_obs.d_right, target_obs.d_left);
        zone.d_hi = std::max(target_obs.d_right, target_obs.d_left);
        zone.s_lo = target_obs.s_start;
        zone.s_hi = target_obs.s_end;

        // 장애물이 실제로 걸친 layer 들.
        const int span = (end_layer - start_layer + nL) % nL;
        for (int k = 0; k <= span; ++k) {
            zone.layers.push_back((start_layer + k) % nL);
        }

        // 그 다음 한 장("꼬리 layer")은 장애물의 s 범위 밖이다. 원래는 장애물을
        // "가로지르는" edge (마지막 layer -> 그 다음 layer) 를 잡으려고 같이 막았는데,
        // 이 layer 에는 장애물의 frenet d 밴드가 그대로 복사된다. 코너에서는
        // raceline 이 트랙 안에서 옆으로 이동하므로 그 밴드가 엉뚱한 쪽을 지우고,
        // 결과적으로 앞뒤 layer 의 자유 구간이 좌우로 갈라져 통로가 끊긴다
        // (obs_debug_0819 실측: layer 30 자유=우측, layer 31 자유=좌측).
        //
        // 장애물을 뚫는 edge 는 어차피 "진짜 layer" 의 차단 노드에서 나가는 edge 라
        // apply_node_filter 가 이미 잡는다. 그래서 기본값은 꼬리 layer 를 막지 않는다.
        // 예전 거동으로 되돌리려면 block_obs_tail_layer:=true (동적 파라미터).
        zone.tail_layer = (start_layer + span + 1) % nL;
        if (block_obs_tail_layer_) zone.layers.push_back(zone.tail_layer);

        return zone;
    }

    // layer 하나에 대해, 주어진 횡방향 여유 margin [m] 으로 차단되는 노드 인덱스 집합.
    // margin = ego 반차폭 + 안전여유. 노드 i 의 ego footprint 가 장애물과 겹치면 차단한다.
    //   노드 i 의 raceline 기준 d = d_sign_ * (i - raceline_idx) * lat_resolution
    //   겹침 <=> d_lo - margin <= d_i <= d_hi + margin
    // 장애물이 여러 개면 각 구간의 합집합이다(사이의 빈 공간은 살아남는다).
    std::set<int> blocked_idx_at_layer(const std::vector<ObsZone> &zones, int layer, double margin) const {
        std::set<int> out;
        const int n_node = static_cast<int>(nodeMap[layer].size());
        const int rl     = nodeIndicesOnRaceline[layer];
        const double res = params.lat_resolution;

        for (const auto &z : zones) {
            if (std::find(z.layers.begin(), z.layers.end(), layer) == z.layers.end()) continue;

            double a = (z.d_lo - margin) / (d_sign_ * res);
            double b = (z.d_hi + margin) / (d_sign_ * res);
            if (a > b) std::swap(a, b);   // d_sign_ 이 음수면 부등호가 뒤집힌다

            const int i0 = std::max(0,          rl + static_cast<int>(std::ceil (a - 1e-9)));
            const int i1 = std::min(n_node - 1, rl + static_cast<int>(std::floor(b + 1e-9)));
            for (int i = i0; i <= i1; ++i) out.insert(i);
        }
        return out;
    }

    // 노드 인덱스가 증가하는 방향이 frenet d 의 +/- 중 어느 쪽인지 실제 변환으로 한 번 판정한다.
    // ltpl 의 normvec 이 오른쪽을 향하면 "인덱스 증가 = d 감소" 지만, 맵 내보내기 규약에 따라
    // 달라질 수 있어 하드코딩하지 않는다. 부호를 틀리면 장애물 반대쪽을 막게 되므로 치명적이다.
    void calibrate_d_sign() {
        for (size_t l = 0; l < nodeMap.size(); ++l) {
            const int rl = nodeIndicesOnRaceline[l];
            const int probe = rl + 1;
            if (probe >= static_cast<int>(nodeMap[l].size())) continue;

            std::vector<double> xs{nodeMap[l][probe].x}, ys{nodeMap[l][probe].y};
            std::vector<double> hint{stMap[RL_S][l]};
            auto sd = converter->get_frenet(xs, ys, &hint);
            if (std::abs(sd.second[0]) < 1e-6) continue;

            d_sign_ = (sd.second[0] < 0.0) ? -1.0 : 1.0;
            RCLCPP_INFO(this->get_logger(),
                "d_sign 판정: 노드 인덱스 증가 = d %s (probe layer %zu, d=%.3f)",
                d_sign_ < 0 ? "감소" : "증가", l, sd.second[0]);
            return;
        }
        RCLCPP_WARN(this->get_logger(), "d_sign 판정 실패, 기본값 %.0f 사용", d_sign_);
    }

    auto findDestination(const vector<std::tuple<int,int,int>> &blocked_zones) -> pair<IPair, IPair> {
        
        // 시작 노드. s/d 가 이미 있으므로 cartesian 변환 없이 바로 레이어를 고른다.
        // 예전에는 get_cartesian 후 getClosestNodes 로 (x,y) 최근접을 썼는데, 코너 안쪽에서
        // 엉뚱한 레이어로 스냅되면 목표 레이어(start + min_plan_horizon)까지 통째로 밀리고,
        // 스냅된 노드가 차단 영역 안이면 graph_search 가 첫 홉에서 끝난다
        // (expanded=1, pushed=0 로 관측됨). layer_lookup_by_s:=false 로 예전 거동 복구.
        IPair startIdx;
        {
            const int l_s = layer_at_s(cur_s);
            IPair by_s = {l_s, node_idx_at_d(l_s, cur_d)};

            if (layer_lookup_by_s_ && !snap_diag_) {
                startIdx = by_s;
            } else {
                auto [x,y] = converter->get_cartesian(cur_s, cur_d);
                IPair by_xy = getClosestNodes(Eigen::Vector2d(x,y), 1);
                if (snap_diag_) {
                    const int gap = layer_gap(by_s.first, by_xy.first);
                    snap_n_++; snap_gap_sum_ += gap;
                    if (gap > 0) snap_mismatch_++;
                    if (gap > snap_gap_max_) snap_gap_max_ = gap;
                }
                startIdx = layer_lookup_by_s_ ? by_s : by_xy;
            }
        }

        // 목적지 거리 계산
        // int max_blocked_layer = startIdx.first;
        // for (auto &[layer, idx_min, idx_max] : blocked_zones) {
        //     if (layer > max_blocked_layer) {
        //         max_blocked_layer = layer;
        //     }
        // }

        // int dest_layer = std::min(max_blocked_layer + 1, (int)nodeMap.size() - 1) + min_plan_horizon_;
        int dest_layer = startIdx.first + min_plan_horizon_;
        // 레이어 wrapping
        if (dest_layer >= (int)nodeIndicesOnRaceline.size()) {
            dest_layer = dest_layer % nodeIndicesOnRaceline.size();
        }

        int dest_index = nodeIndicesOnRaceline[dest_layer];
        IPair endIdx = {dest_layer, dest_index};
        // RCLCPP_INFO(this->get_logger(),
        //     "[findDestination] start=(layer=%d, idx=%d), dest=(layer=%d, idx=%d)",
        //     startIdx.first, startIdx.second,
        //     endIdx.first, endIdx.second
        // );
        return {startIdx, endIdx};
    }


    visualization_msgs::msg::Marker xy_to_point(double x, double y, bool opponent=true) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
        m.color.a = 0.8;
        m.color.b = 0.65;
        m.color.r = opponent ? 1.0 : 0.0;
        m.color.g = 0.65;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.01;
        m.pose.orientation.w = 1.0;
        return m;
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
     
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleSpliner>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}