#pragma once
#include "graph_planner.hpp"
#include "offline_params.hpp"
#include <set>
#include <map>

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        // 간단한 해시 조합
        return h1 ^ (h2 << 1);
    }
};

struct pair2_hash {
    std::size_t operator()(const std::pair<std::pair<int,int>, std::pair<int,int>>& p) const noexcept {
        auto h1 = pair_hash{}(p.first);
        auto h2 = pair_hash{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

class NodeGraph {
private:
  IPairAdjList nodeGraph;
  IPairAdjList nodeGraphOrig;   
  SplineMap splineMap;
  SplineMap splineMapOrig;
  int num_layers;
  OfflineParams params;

public:
  std::unordered_map<std::pair<IPair, IPair>, double, pair2_hash> orig_edges;
  void setParams(const OfflineParams &p) { params = p; }

  void setNumLayers(NodeMap& nodeMap) {
      num_layers = static_cast<int>(nodeMap.size());
  }

  SplineMap &getSplineMap() { return splineMap; }

  SplineInfo &at(const IPair &start, const IPair &end)  {
    return splineMap[start][end];
  }

  const SplineInfo &at(const IPair &start, const IPair &end) const  {
    return splineMap.at(start).at(end);
  }

  void addEdge(IPair srcIdx, IPair dstIdx)  {
    nodeGraph[srcIdx].push_back(dstIdx);
  }

  void printGraph(rclcpp::Logger logger)  {
    int size = 0;
    for (const auto &[srcNode, childNode] : nodeGraph)    {
      // cout << "(" << srcNode.first << "," << srcNode.second << ")" << ": ";
      for (size_t i = 0; i < childNode.size(); ++i)      {
        // cout << "(" << childNode[i].first << ", " << childNode[i].second << ")" << " -> ";
        size++;
      }
      // cout << "NULL\n";
    }
    // for (const auto& [key, neighbors] : nodeGraph) {
    //     cout << "(" << get<0>(key) << "," << get<1>(key) << ")" << ": ";
    //     for (int dest : neighbors) {
    //         cout << dest << " -> ";
    //     }
    //     cout << "NULL\n";
    // }
    RCLCPP_INFO(logger, "[INFO] Total number of splines =%d", size);
  }

  void printsize()  {
    int size = 0;
    for (const auto &[srcNode, childNode] : nodeGraph)    {
      // cout << "(" << srcNode.first << "," << srcNode.second << ")" << ": ";
      for (size_t i = 0; i < childNode.size(); ++i)      {
        // cout << "(" << childNode[i].first << ", " << childNode[i].second << ")" << " -> ";
        size++;
      }
      // cout << "NULL\n";
    }
    // for (const auto& [key, neighbors] : nodeGraph) {
    //     cout << "(" << get<0>(key) << "," << get<1>(key) << ")" << ": ";
    //     for (int dest : neighbors) {
    //         cout << dest << " -> ";
    //     }
    //     cout << "NULL\n";
    // }
    cout << size;
  }

  IPairVector getChildList(const IPair &srcIdx)  {
    IPairVector childList;
    for (auto &value : nodeGraph[srcIdx])    {
      childList.push_back(value);
      // cout << "("<< value.first << ", " << value.second << ")" << endl;
    }
    return childList;
  }

  IPairVector getParentList(const IPair &srcIdx) {
      IPairVector parentList;
      for (auto &[key, vec] : nodeGraph) {
          // 순환 구조 처리: 첫 번째 레이어의 부모는 마지막 레이어
          if (srcIdx.first == 0) {
              if (key.first == num_layers - 1) {  // == 로 수정
                  for (const auto &value : vec) {
                      if (value == srcIdx)
                          parentList.push_back(key);
                  }
              }
          }
          // 일반적인 경우: 이전 레이어가 부모
          else if (key.first == (srcIdx.first) - 1) {
              for (const auto &value : vec) {
                  if (value == srcIdx)
                      parentList.push_back(key);
              }
          }
      }
      return parentList;
  }


  void removeEdge(const IPair &srcIdx, const IPair &dstIdx)  {
      IPairVector &childs = nodeGraph[srcIdx];
      auto it = remove(childs.begin(), childs.end(), dstIdx);
      if (it != childs.end()) {
          childs.erase(it, childs.end());
      } else {
          return;
      }

      auto itSpline = splineMap.find(srcIdx);
      if (itSpline != splineMap.end()) {
          auto &splineList = itSpline->second;
          auto it2 = splineList.find(dstIdx);
          if (it2 != splineList.end()) {
              splineList.erase(it2);
              if (splineList.empty()) {
                  splineMap.erase(itSpline);
              }
          }
      }

      if (childs.empty()) {
          IPairVector parentList = getParentList(srcIdx);
          if (!parentList.empty()) {
              for (const auto &parentIdx : parentList) {
                  removeEdge(parentIdx, srcIdx);
              }
          }
      }
      
      IPairVector dstParents = getParentList(dstIdx);
      if (dstParents.empty()) {
        IPairVector dstChilds = nodeGraph[dstIdx];
        for (const auto &dstChild : dstChilds) {
          removeEdge(dstIdx, dstChild);
        }
      }
  }

  void writeSplineMapToCSV(const std::string &filename, rclcpp::Logger logger) {
      RCLCPP_INFO(logger, "[INFO] Path to splineMap: %s", filename.c_str());
      std::ofstream fout(filename);
      if (!fout.is_open())
          throw std::runtime_error("Cannot open file");
        
      // for (auto& [sp, ep_map] : splineMap) {
      //   for (auto& [ep, spline] : ep_map) {
      //       RCLCPP_INFO(logger, "Edge (%d,%d)->(%d,%d) kappaVector.size()=%zu",
      //                   sp.first, sp.second, ep.first, ep.second, spline.kappaVector.size());
      //     }
      // }
          // 헤더
      fout << "start_layer,start_idx,end_layer,end_idx,"
              "coeffs_x(a0 a1 a2 a3),coeffs_y(b0 b1 b2 b3),"
              "avg_kappa,max_kappa,length,raceline\n";

      for (const auto &[start, endMap] : splineMap) {
          for (const auto &[end, spline] : endMap) {
              fout << start.first << "," << start.second << ","
                  << end.first << "," << end.second << ",";

              // coeffs_x
              fout << "[";
              for (int j = 0; j < spline.coeffs_x.cols(); ++j) {
                  fout << spline.coeffs_x(0, j);
                  if (j < spline.coeffs_x.cols() - 1) fout << " ";
              }
              fout << "],";

              // coeffs_y
              fout << "[";
              for (int j = 0; j < spline.coeffs_y.cols(); ++j) {
                  fout << spline.coeffs_y(0, j);
                  if (j < spline.coeffs_y.cols() - 1) fout << " ";
              }
              fout << "],";

              // 곡률 요약 (평균 / 최대)
              double avg_kappa = 0.0;
              double max_kappa = 0.0;
              if (spline.kappaVector.size() > 0) {
                  avg_kappa = spline.kappaVector.array().abs().mean();
                  max_kappa = spline.kappaVector.array().abs().maxCoeff();
              }

              // spline 길이
              double length = (spline.el_lengths.size() > 0) ? spline.el_lengths.sum() : 0.0;

              fout << avg_kappa << "," << max_kappa << "," << length << ",";

              fout << (spline.raceline ? 1 : 0) << "\n";
          }
      }
      fout.close();
  }


  void readSplineMapFromCSV(const string &filename)
  {
    ifstream fin(filename);
    if (!fin.is_open())
      throw runtime_error("Cannot open file");

    string line;
    getline(fin, line); // 헤더 스킵

    while (getline(fin, line))    {
      stringstream ss(line);
      string item;

      IPair start, end;
      SplineInfo spline;

      // start_layer, start_idx, end_layer, end_idx
      getline(ss, item, ',');
      start.first = stoi(item);
      getline(ss, item, ',');
      start.second = stoi(item);
      getline(ss, item, ',');
      end.first = stoi(item);
      getline(ss, item, ',');
      end.second = stoi(item);

      // coeffs_x
      getline(ss, item, ',');
      stringstream sx(item);
      DVector vx;
      double val;
      while (sx >> val)
        vx.push_back(val);

      spline.coeffs_x = MatrixXd(4, 1);
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 1; j++)
          spline.coeffs_x(i, j) = vx[i * 1 + j];

      // coeffs_y
      getline(ss, item, ',');
      stringstream sy(item);
      DVector vy;
      while (sy >> val)
        vy.push_back(val);
      spline.coeffs_y = MatrixXd(4, 1);
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 1; j++)
          spline.coeffs_y(i, j) = vy[i * 1 + j];

      // kappa
      getline(ss, item, ',');
      stringstream sk(item);
      DVector vk;
      while (sk >> val)
        vk.push_back(val);
      spline.kappaVector = VectorXd::Map(vk.data(), vk.size());

      // points_xy
      getline(ss, item, ',');
      stringstream sp(item);
      vector<Vector2d> pts;
      double x, y;
      while (sp >> x >> y)
        pts.emplace_back(x, y);
      spline.points_xy = pts;

      // raceline
      getline(ss, item, ',');
      spline.raceline = (stoi(item) != 0);

      splineMap[start][end] = spline;
    }

    fin.close();
  }

  auto computeSplines(const MatrixXd &path,
                    double psi_s,
                    double psi_e,
                    bool use_dist_scaling=true) -> std::unique_ptr<SplineInfo> {
      // 구간 길이 계산
      VectorXd el_lengths;
      if (use_dist_scaling)
      {
          el_lengths.resize(path.rows() - 1);
          for (int i = 0; i < path.rows() - 1; ++i)
          {
              el_lengths(i) = (path.row(i + 1) - path.row(i)).norm();
          }
      }
      // 맨 마지막 거리 추가
      if (use_dist_scaling && el_lengths.size() > 0)
      {
          VectorXd el_tmp(el_lengths.size() + 1);
          el_tmp << el_lengths, el_lengths(0);
          el_lengths = el_tmp;
      }

      int no_splines = path.rows() - 1;
      if (no_splines <= 0) {
          RCLCPP_WARN(rclcpp::get_logger("graph_planner"),
                      "computeSplines: path too short (rows=%ld). Returning empty spline.",
                      path.rows());
          return std::make_unique<SplineInfo>(SplineInfo{
              MatrixXd(), MatrixXd(),
              VectorXd(), VectorXd(),
              {}, 0.0, false
          });
      }

      // 도함수 스케일링
      VectorXd scaling = VectorXd::Ones(no_splines - 1);
      if (use_dist_scaling && no_splines > 1)
      {
          for (int i = 0; i < no_splines - 1; ++i)
          {
              scaling(i) = el_lengths(i) / el_lengths(i + 1);
          }
      }

      MatrixXd M = MatrixXd::Zero(no_splines * 4, no_splines * 4);
      VectorXd b_x = VectorXd::Zero(no_splines * 4);
      VectorXd b_y = VectorXd::Zero(no_splines * 4);

      // 연속 조건 template
      Matrix<double, 4, 8> template_M;
      template_M << 1, 0, 0, 0, 0, 0, 0, 0,
                    1, 1, 1, 1, 0, 0, 0, 0,
                    0, 1, 2, 3, 0, -1, 0, 0,
                    0, 0, 2, 6, 0, 0, -2, 0;

      // spline 구간별 행렬 세팅
      for (int i = 0; i < no_splines; ++i)
      {
          int j = i * 4;
          if (i < no_splines - 1)
          {
              M.block(j, j, 4, 8) = template_M;
              M(j + 2, j + 5) *= scaling(i);
              M(j + 3, j + 6) *= pow(scaling(i), 2);
          }
          else
          {
              M.block(j, j, 2, 4) << 1, 0, 0, 0,
                                      1, 1, 1, 1;
          }

          b_x.segment(j, 2) << path(i, 0), path(i + 1, 0);
          b_y.segment(j, 2) << path(i, 1), path(i + 1, 1);
      }

      // 시작/끝점에서의 psi 반영
      psi_s += M_PI_2;
      psi_e += M_PI_2;

      M(no_splines * 4 - 2, 1) = 1.0;
      double el_length_s = el_lengths.size() > 0 ? el_lengths(0) : 1.0;
      b_x(no_splines * 4 - 2) = cos(psi_s) * el_length_s;
      b_y(no_splines * 4 - 2) = sin(psi_s) * el_length_s;

      M.block(no_splines * 4 - 1, no_splines * 4 - 4, 1, 4) << 0, 1, 2, 3;
      double el_length_e = el_lengths.size() > 0 ? el_lengths.tail(1)(0) : 1.0;
      b_x(no_splines * 4 - 1) = cos(psi_e) * el_length_e;
      b_y(no_splines * 4 - 1) = sin(psi_e) * el_length_e;

      // 선형시스템 풀기
      VectorXd x_les = M.fullPivLu().solve(b_x);
      VectorXd y_les = M.fullPivLu().solve(b_y);

      if (x_les.size() != no_splines * 4 || y_les.size() != no_splines * 4) {
          throw std::runtime_error("computeSplines: solution size mismatch");
      }

      // 🔑 reshape: (no_splines x 4)
      MatrixXd coeffs_x = Eigen::Map<const Matrix<double,4,Eigen::Dynamic>>(x_les.data(), 4, no_splines).transpose();
      MatrixXd coeffs_y = Eigen::Map<const Matrix<double,4,Eigen::Dynamic>>(y_les.data(), 4, no_splines).transpose();

      // 결과 반환
      return std::make_unique<SplineInfo>(SplineInfo{
          coeffs_x,
          coeffs_y,
          VectorXd(),     // kappaVector (추후 계산)
          el_lengths,
          {},             // points_xy
          0.0,            // cost
          false           // raceline flag
      });
  }


  void genEdges(NodeMap &nodeMap,
                const IVector &nodeIndexesOnRaceline,
                rclcpp::Logger logger) {

    if (params.lat_offset <= 0.0)    {
      throw invalid_argument("Too small lateral offset!");
    }
    // ---- genEdges 실측 계측 (2026-08-20). 오프라인 1회만 도므로 비용 무시 가능 ----
    size_t dbg_pairs_possible = 0;   // 이론상 가능한 (src,dst) 쌍 = sum(n_L * n_{L+1})
    size_t dbg_pairs_tried    = 0;   // lat_steps 창 안에 들어와 실제로 시도한 쌍
    size_t dbg_edges_added    = 0;   // addEdge 호출 수
    size_t dbg_spline_empty   = 0;   // computeSplines 가 빈 결과를 준 횟수
    std::map<int,int> dbg_lat_steps_hist;
    std::map<int,int> dbg_window_hist;  // 창 폭(실제 연결 가능 노드 수) 분포
    for (int l = 0; l < num_layers; ++l) {
      int d = (l + 1) % num_layers;
      dbg_pairs_possible += nodeMap[l].size() * nodeMap[d].size();
    }
    // for (int i=0; i<num_layers; ++i) {
    //     RCLCPP_INFO(logger, "layer %d, nodeMap size=%zu, nodeIndexOnRaceline=%d", i, nodeMap[i].size(), nodeIndexesOnRaceline[i]);
    // }
    // raceline
    for (int layerIdx = 0; layerIdx < num_layers; ++layerIdx)    {
      int dstLayerIdx = layerIdx + 1;
      if (dstLayerIdx >= num_layers)      {
        dstLayerIdx -= num_layers;
      }
      int startNodeIdx = nodeIndexesOnRaceline[layerIdx];
      int endNodeIdx = nodeIndexesOnRaceline[dstLayerIdx];
      // RCLCPP_INFO(logger, "layerIdx=%d, startNodeIdx=%d, endNodeIdx=%d", layerIdx, startNodeIdx, endNodeIdx);

      ::Node &startNode = nodeMap[layerIdx][startNodeIdx];
      ::Node &endNode = nodeMap[dstLayerIdx][endNodeIdx];

      MatrixXd path(2, 2);
      path(0, 0) = startNode.x;
      path(0, 1) = startNode.y;
      path(1, 0) = endNode.x;
      path(1, 1) = endNode.y;

      auto result = computeSplines(path, startNode.psi, endNode.psi);
      if (!result) {
        RCLCPP_ERROR(logger, "computeSplines returned nullptr for layerIdx=%d", layerIdx);
      continue;
}

      IPair startPoint = make_pair(layerIdx, startNodeIdx);
      IPair endPoint = make_pair(dstLayerIdx, endNodeIdx);

      result->raceline = true;

      splineMap[startPoint][endPoint] = *result;

      addEdge(startPoint, endPoint);
    }

    // cout << num_layers << endl; // 출력: 51
    // 레이어 별 loop
    // raceline spline 먼저 생성해서 splineMap에 등록, 뒤에서 재등록하지 않게 index 겹치면 pass
    for (int layerIdx = 0; layerIdx < num_layers; ++layerIdx)    {

      int srcLayerIdx = layerIdx;
      int dstLayerIdx = srcLayerIdx + 1;

      // cout << "srcLayerIdx:" << srcLayerIdx << endl;
      // cout << "num_layers" << num_layers << endl;

      // 마지막 layer의 경우 0번째 layer와 연결시킬 수 있도록 dstLayerIdx 조정
      if (dstLayerIdx >= num_layers)      {
        dstLayerIdx -= num_layers;
      }

      // start layer 내 노드별 loop
      for (size_t srcNodeIdx = 0; srcNodeIdx < nodeMap[srcLayerIdx].size(); ++srcNodeIdx)      {
        // 기준 노드
        Node &startNode = nodeMap[srcLayerIdx][srcNodeIdx];

        int refEndNodeIdx = nodeIndexesOnRaceline[dstLayerIdx] - (nodeIndexesOnRaceline[srcLayerIdx] - srcNodeIdx);
        refEndNodeIdx = max(0, min(refEndNodeIdx, static_cast<int>(nodeMap[dstLayerIdx].size() - 1)));
        // refEndNodeIdx = clamp(refEndNodeIdx, 0, static_cast<int>(nodeMap[dstLayerIdx].size() - 1));
        // int refEndNodeIdx = srcNodeIdx;
        // int refEndNodeIdx = nodeIndexesOnRaceline[dstLayerIdx];
        Node &endNode = nodeMap[dstLayerIdx][refEndNodeIdx];

        Vector2d d_start(startNode.x, startNode.y);
        Vector2d d_end(endNode.x, endNode.y);

        // spline 연결할 노드 선정 기준 : lat_steps
        double dist = (d_end - d_start).norm();
        // genNode에서 kappa 계산한거 토대로(+기능 추가 완료)

        int lat_steps = static_cast<int>(round(dist * params.lat_offset / params.lat_resolution));
        // cout << srcLayerIdx << "의 " << srcNodeIdx << "가 다음 refendNode와의 거리: " << dist << endl;
        lat_steps = min(lat_steps, params.max_lat_steps); // endNode 기준 2*lat_steps + 1개의 노드와 연결한다.
        dbg_lat_steps_hist[lat_steps]++;
        {
          int lo = max(0, refEndNodeIdx - lat_steps);
          int hi = min(static_cast<int>(nodeMap[dstLayerIdx].size() - 1), refEndNodeIdx + lat_steps);
          dbg_window_hist[max(0, hi - lo + 1)]++;
        }
        // cout << srcNodeIdx << "번째 노드의 lat_steps" << lat_steps << endl;
        // startNode와 lat_steps 기준 해당되는 노드들 spline 연결
        for (int endNodeIdx = max(0, refEndNodeIdx - lat_steps);
             endNodeIdx <= min(static_cast<int>(nodeMap[dstLayerIdx].size() - 1), refEndNodeIdx + lat_steps); ++endNodeIdx)        {

          if (srcNodeIdx == nodeIndexesOnRaceline[layerIdx] && endNodeIdx == nodeIndexesOnRaceline[dstLayerIdx])          {
            continue;
          }

          Node &endNode = nodeMap[dstLayerIdx][endNodeIdx];

          MatrixXd path(2, 2);
          path(0, 0) = startNode.x;
          path(0, 1) = startNode.y;
          path(1, 0) = endNode.x;
          path(1, 1) = endNode.y;

          dbg_pairs_tried++;
          auto result = computeSplines(path, startNode.psi, endNode.psi);
          // cout << "result: " << result->el_lengths.size()  << endl;
          if (!result || result->coeffs_x.size() == 0) dbg_spline_empty++;
          IPair startPoint = make_pair(srcLayerIdx, srcNodeIdx);
          IPair endPoint = make_pair(dstLayerIdx, endNodeIdx);

          splineMap[startPoint][endPoint] = *result;

          // graph에 넣는 과정
          addEdge(startPoint, endPoint);
          dbg_edges_added++;

          // cout << "startPoint:" << startPoint.first << ", " << startPoint.second << " -> ";
          // cout << "endPoint:" << endPoint.first << ", " << endPoint.second << endl;
        }
      }
    }

    // ---- 실측 리포트 ----------------------------------------------------
    {
      size_t total_nodes = 0, zero_child = 0, min_ch = SIZE_MAX, max_ch = 0, sum_ch = 0;
      std::map<size_t,int> child_hist;
      std::ostringstream layers;
      for (int l = 0; l < num_layers; ++l) {
        total_nodes += nodeMap[l].size();
        for (size_t i = 0; i < nodeMap[l].size(); ++i) {
          size_t c = nodeGraph[{l, (int)i}].size();
          child_hist[c]++;
          sum_ch += c;
          if (c == 0) zero_child++;
          min_ch = std::min(min_ch, c);
          max_ch = std::max(max_ch, c);
        }
        layers << l << ":" << nodeMap[l].size() << "/rl" << nodeIndexesOnRaceline[l] << " ";
      }
      std::ostringstream ls, ws, cs;
      for (auto &kv : dbg_lat_steps_hist) ls << kv.first << "->" << kv.second << " ";
      for (auto &kv : dbg_window_hist)    ws << kv.first << "->" << kv.second << " ";
      for (auto &kv : child_hist)         cs << kv.first << "->" << kv.second << " ";

      RCLCPP_INFO(logger,
        "\n===== genEdges 실측 =====\n"
        "  layer %d개, 노드 %zu개\n"
        "  이론상 가능한 (src,dst) 쌍 sum(n_L*n_L+1) = %zu\n"
        "  lat_steps 창 안에서 시도한 쌍           = %zu  (%.1f%%)\n"
        "  실제 addEdge 된 엣지                    = %zu  (raceline %d개 별도)\n"
        "  computeSplines 빈 결과                  = %zu\n"
        "  노드당 자식: min %zu / 평균 %.2f / max %zu, 자식 0개인 노드 %zu개\n"
        "  자식수 분포      : %s\n"
        "  lat_steps 분포   : %s\n"
        "  연결창 폭 분포   : %s\n"
        "  layer:노드수/rl  : %s",
        num_layers, total_nodes, dbg_pairs_possible, dbg_pairs_tried,
        100.0 * dbg_pairs_tried / std::max<size_t>(dbg_pairs_possible,1),
        dbg_edges_added, num_layers, dbg_spline_empty,
        (min_ch==SIZE_MAX?0:min_ch), (double)sum_ch/std::max<size_t>(total_nodes,1), max_ch, zero_child,
        cs.str().c_str(), ls.str().c_str(), ws.str().c_str(), layers.str().c_str());
    }
  }

std::vector<SplineSample> interpSpline(const MatrixXd &coeffs_x,
                                       const MatrixXd &coeffs_y,
                                       double resolution = 0.1)
{
    if (coeffs_x.cols() != 4 || coeffs_y.cols() != 4) {
        throw std::invalid_argument("interpSpline: coeffs must have 4 columns per segment");
    }

    std::vector<SplineSample> out;
    int no_splines = coeffs_x.rows();

    // 1) arc length 계산용 dense 샘플링
    std::vector<double> cum_s; 
    std::vector<SplineSample> dense_samples;
    double total_length = 0.0;

    int dense_per_seg = 100;
    for (int seg = 0; seg < no_splines; ++seg) {
        double step = 1.0 / (dense_per_seg - 1);
        for (int i = 0; i < dense_per_seg; ++i) {
            double t  = i * step;
            double t2 = t*t;
            double t3 = t2*t;

            double x   = coeffs_x(seg,0) + coeffs_x(seg,1)*t + coeffs_x(seg,2)*t2 + coeffs_x(seg,3)*t3;
            double y   = coeffs_y(seg,0) + coeffs_y(seg,1)*t + coeffs_y(seg,2)*t2 + coeffs_y(seg,3)*t3;
            double x_d = coeffs_x(seg,1) + 2*coeffs_x(seg,2)*t + 3*coeffs_x(seg,3)*t2;
            double y_d = coeffs_y(seg,1) + 2*coeffs_y(seg,2)*t + 3*coeffs_y(seg,3)*t2;
            double x_dd= 2*coeffs_x(seg,2) + 6*coeffs_x(seg,3)*t;
            double y_dd= 2*coeffs_y(seg,2) + 6*coeffs_y(seg,3)*t;

            double denom = std::pow(x_d*x_d + y_d*y_d, 1.5);
            double kappa = (denom > 1e-9) ? (x_d*y_dd - y_d*x_dd)/denom : 0.0;

            SplineSample sample{x,y,x_d,y_d,x_dd,y_dd,kappa};

            if (!dense_samples.empty()) {
                double dx = x - dense_samples.back().x;
                double dy = y - dense_samples.back().y;
                total_length += std::hypot(dx,dy);
            }

            cum_s.push_back(total_length);
            dense_samples.push_back(sample);
        }
    }

    if (total_length < 1e-6) {
        throw std::runtime_error("interpSpline: total length is zero");
    }

    double usable_length = total_length;

    int n_points = std::max(2, static_cast<int>(std::round(usable_length / resolution)));
    out.reserve(n_points);

    for (int k = 0; k < n_points; ++k) {
        double target_s = (static_cast<double>(k) / (n_points - 1)) * usable_length;

        auto it = std::lower_bound(cum_s.begin(), cum_s.end(), target_s);
        int idx = std::distance(cum_s.begin(), it);

        if (idx == 0) {
            out.push_back(dense_samples[0]);
        } else if (idx >= (int)cum_s.size()) {
            out.push_back(dense_samples.back());
        } else {
            double s0 = cum_s[idx-1], s1 = cum_s[idx];
            double alpha = (target_s - s0) / (s1 - s0);
            const auto &p0 = dense_samples[idx-1];
            const auto &p1 = dense_samples[idx];

            SplineSample interp;
            interp.x    = (1-alpha)*p0.x    + alpha*p1.x;
            interp.y    = (1-alpha)*p0.y    + alpha*p1.y;
            interp.x_d  = (1-alpha)*p0.x_d  + alpha*p1.x_d;
            interp.y_d  = (1-alpha)*p0.y_d  + alpha*p1.y_d;
            interp.x_dd = (1-alpha)*p0.x_dd + alpha*p1.x_dd;
            interp.y_dd = (1-alpha)*p0.y_dd + alpha*p1.y_dd;
            interp.kappa= (1-alpha)*p0.kappa+ alpha*p1.kappa;
            out.push_back(interp);
        }
    }

    return out;
}
  auto sampleSingleSpline(MatrixXd &coeffs_x, MatrixXd &coeffs_y)
      -> std::pair<std::vector<Vector2d>, VectorXd> {

      if (coeffs_x.rows() != coeffs_y.rows()) {
          throw std::invalid_argument("Coefficient matrices must have the same length!");
      }

      if (coeffs_x.cols() < 4 || coeffs_y.cols() < 4) {
          throw std::invalid_argument("Coefficient matrices must each have 4 coefficients per spline!");
      }

      const int N = params.no_interp_points;
      VectorXd t_steps(N);
      const double step = 1.0 / (N - 1);
      for (int i = 0; i < N; ++i) {
          t_steps(i) = i * step;
      }

      std::vector<Vector2d> points_xy;
      points_xy.reserve(N);
      VectorXd kappaVector(N);
      kappaVector.setZero();

      for (int i = 0; i < N; ++i) {
          double t = t_steps(i);
          double t2 = t * t;
          double t3 = t2 * t;

          // 좌표
          double x = coeffs_x(0, 0) + coeffs_x(0, 1)*t + coeffs_x(0, 2)*t2 + coeffs_x(0, 3)*t3;
          double y = coeffs_y(0, 0) + coeffs_y(0, 1)*t + coeffs_y(0, 2)*t2 + coeffs_y(0, 3)*t3;

          // 1차 미분
          double dx = coeffs_x(0, 1) + 2*coeffs_x(0, 2)*t + 3*coeffs_x(0, 3)*t2;
          double dy = coeffs_y(0, 1) + 2*coeffs_y(0, 2)*t + 3*coeffs_y(0, 3)*t2;

          // 2차 미분
          double ddx = 2*coeffs_x(0, 2) + 6*coeffs_x(0, 3)*t;
          double ddy = 2*coeffs_y(0, 2) + 6*coeffs_y(0, 3)*t;

          double denom = std::pow(dx*dx + dy*dy, 1.5);

          double kappa = 0.0;
          if (denom > 1e-9)  // 너무 작은 값 방지
              kappa = (dx * ddy - dy * ddx) / denom;

          kappaVector(i) = kappa;
          points_xy.emplace_back(x, y);
      }

      // 후처리:NaN 제거 및 미세 스무딩
      for (int i = 0; i < N; ++i) {
          if (!std::isfinite(kappaVector(i))) {
              kappaVector(i) = 0.0;
          }
      }

      // (선택사항) 너무 noise한 경우 moving average
      for (int i = 1; i < N - 1; ++i) {
          kappaVector(i) = 0.25*kappaVector(i-1) + 0.5*kappaVector(i) + 0.25*kappaVector(i+1);
      }
      
      return {points_xy, kappaVector};
  }


  ///////////////////////////////////////////////////////////////////
  /////////////////////////////제거 과정///////////////////////////////
  ///////////////////////////////////////////////////////////////////

    // 계측용: layer L -> L+1 로 남아있는 엣지 수
    std::vector<size_t> edgeCountPerLayer(NodeMap &nodeMap) {
        std::vector<size_t> out(num_layers, 0);
        for (int l = 0; l < num_layers; ++l)
            for (size_t i = 0; i < nodeMap[l].size(); ++i)
                out[l] += nodeGraph[{l, (int)i}].size();
        return out;
    }

    void pruneEdges(NodeMap &nodeMap, const DVector& raceline_vx)  {

      int rmv_cnt = 0;
      // ---- pruneEdges 실측 (2026-08-20) ----
      int dbg_curv_rm = 0, dbg_isolated_rm = 0;
      std::map<int,int> dbg_curv_by_layer, dbg_iso_by_layer;
      double dbg_kappa_max_seen = 0.0;
      std::map<int,int> dbg_kappa_hist;
      auto dbg_before = edgeCountPerLayer(nodeMap);
      
      // s_time = clock();

      for (size_t layer_idx = 0; layer_idx < num_layers;++layer_idx) {
        int srcLayerIdx = layer_idx;
        for (size_t node_idx = 0; node_idx < nodeMap[srcLayerIdx].size(); ++node_idx) {

          IPair start = make_pair(layer_idx ,node_idx);
          IPairVector childList = getChildList(start);

          // 연결된 노드와 loop
          for (auto& end : childList) {
              MatrixXd& coeffs_x = splineMap[start][end].coeffs_x;
              MatrixXd& coeffs_y = splineMap[start][end].coeffs_y;
              // spline 위의 점들을 샘플링(no_interp_points개수만큼)
              auto [points_xy, kappaVector] = sampleSingleSpline(coeffs_x, coeffs_y);
              // 점들을 기준으로 pruneEdges에 가서 1.곡률 2.트랙내 여부 에 따라 remove를 한다.
              if (kappaVector.size() == 0 || points_xy.size() == 0) {
                  // cerr << "Invalid spline sampling" << endl;
                  continue;
              }
              // 해당 spline위에서 샘플링한 점이 track을 벗어나면 pruneEdges()에 갈 수 있도록.
              splineMap[start][end].kappaVector = kappaVector;
              splineMap[start][end].points_xy = points_xy;

            if (!splineMap[start][end].raceline) {

                int layer_idx = start.first;
                double vel_rl = raceline_vx[layer_idx] * params.min_vel_race;
                double min_turn = pow(vel_rl, 2) / params.max_lateral_accel;

                bool toRemove = false;

                double dbg_peak = kappaVector.array().abs().maxCoeff();
                dbg_kappa_max_seen = std::max(dbg_kappa_max_seen, dbg_peak);
                dbg_kappa_hist[(int)std::floor(std::min(dbg_peak, 20.0))]++;
                for (int j = 0; j < kappaVector.size(); ++j) {

                    double kappa_val = abs(kappaVector(j));
                    // cout << "kappa_val " << kappa_val << endl;
                    // if ((kappa_val > 1.0 / params.veh_turn || kappa_val > 1.0 / min_turn))
                    // {
                    //     toRemove = true;
                    //     break;
                    // }
                    if ((kappa_val > params.prune_kappa_max))
                    {
                        toRemove = true;
                        break;
                    }
                }
                        


                if (toRemove) {
                    removeEdge(start, end);
                    rmv_cnt++;
                    dbg_curv_rm++; dbg_curv_by_layer[start.first]++;
                    }
                }
            }
        }
    }
  

    for (int layerIdx = 0; layerIdx < num_layers; ++layerIdx) {
      for (int nodeIdx = 0; nodeIdx < nodeMap[layerIdx].size(); ++nodeIdx) {
        IPair srcNodeIdx = make_pair(layerIdx, nodeIdx);

        IPairVector parentList = getParentList(srcNodeIdx);

        if (parentList.empty())        {
          // cout << layerIdx << ", " << nodeIdx << endl;
          // cout << "-------" << endl;
          IPairVector childList = getChildList(srcNodeIdx);
          if (!childList.empty())          {
            // cout << layerIdx << ", " << nodeIdx << endl;
            for (auto &child : childList)            {
              // cout << "remove!" << endl;
              removeEdge(srcNodeIdx, child);
              rmv_cnt++;
              dbg_isolated_rm++; dbg_iso_by_layer[layerIdx]++;
            }
          }
        }
      }
    }
    
    if (rmv_cnt > 0)
      cout << "Removed splines due to curvature conditions && isolated nodes: " << rmv_cnt << endl;

    // ---- pruneEdges 실측 리포트 ----
    {
      auto after = edgeCountPerLayer(nodeMap);
      size_t tot_b = 0, tot_a = 0, zero_child = 0, thin_layers = 0;
      std::ostringstream per_layer, kh, ch, ih;
      for (int l = 0; l < num_layers; ++l) { tot_b += dbg_before[l]; tot_a += after[l]; }
      for (int l = 0; l < num_layers; ++l) {
        if (after[l] * 2 < dbg_before[l]) thin_layers++;
        per_layer << l << ":" << dbg_before[l] << "->" << after[l]
                  << (after[l] * 2 < dbg_before[l] ? "!! " : " ");
      }
      for (int l = 0; l < num_layers; ++l)
        for (size_t i = 0; i < nodeMap[l].size(); ++i)
          if (nodeGraph[{l, (int)i}].empty()) zero_child++;
      for (auto &kv : dbg_kappa_hist)     kh << kv.first << "->" << kv.second << " ";
      for (auto &kv : dbg_curv_by_layer)  ch << kv.first << ":" << kv.second << " ";
      for (auto &kv : dbg_iso_by_layer)   ih << kv.first << ":" << kv.second << " ";
      std::cout << "\n===== pruneEdges 실측 =====\n"
        << "  엣지 " << tot_b << " -> " << tot_a
        << "  (" << (tot_b - tot_a) << "개 삭제, " << (100.0*(tot_b-tot_a)/std::max<size_t>(tot_b,1)) << "%)\n"
        << "  명시적 removeEdge 호출: 곡률 " << dbg_curv_rm << " + 고아노드 " << dbg_isolated_rm
        << " = " << rmv_cnt << "  -> 나머지 " << ((long)(tot_b-tot_a) - rmv_cnt) << " 개는 재귀 연쇄 삭제\n"
        << "  최대 |kappa| 관측: " << dbg_kappa_max_seen << " (임계 5.0 = 반경 0.2 m)\n"
        << "  엣지 peak|kappa| 분포 (floor): " << kh.str() << "\n"
        << "  곡률삭제 layer 분포: " << ch.str() << "\n"
        << "  고아삭제 layer 분포: " << ih.str() << "\n"
        << "  자식 0개 노드: " << zero_child << " 개\n"
        << "  절반 이상 날아간 layer: " << thin_layers << " 개\n"
        << "  layer별 엣지 before->after: " << per_layer.str() << "\n" << std::endl;
    }

  }

  void computeSplineCost(IVector &nodeIndexesOnRaceline)  {
    if (splineMap.size() <= 0)    {
      throw invalid_argument("SplineMap's Size is zero!!");
    }

    for (auto &[startPoint, endPoints] : splineMap)    {
      for (auto &[endPoint, spline] : endPoints)      {
        double offline_cost = 0.0;
        int end_layer = endPoint.first;
        int end_node = endPoint.second;

        // 디버깅용
        // cout << "kappa: ";
        // for (int i = 0; i < spline.kappa.size(); ++i) cout << spline.kappa[i] << " ";
        // cout << endl;

        if (end_layer < 0 || end_layer >= nodeIndexesOnRaceline.size())        {
          cerr << "[WARNNING] Skipping spline: end_layer=" << end_layer
               << " out of bounds (0.." << nodeIndexesOnRaceline.size() - 1 << ")\n";
          continue;
        }

        if (spline.kappaVector.size() == 0)        {          
          // cerr << "[WARNNING] Skipping spline: empty curvature data\n";
          continue;
        }

        double abs_kappa = spline.kappaVector.array().abs().sum();
        double s_length = spline.el_lengths.sum();
        // cout << "s_length: " << s_length << endl;
        double side_factor = (nodeIndexesOnRaceline[end_layer] - end_node) > 0 ? 0.7 : 1.2;
        offline_cost += params.w_curv_avg * pow(abs_kappa / spline.kappaVector.size(), 2) * s_length * side_factor;

        // average curvature
        // offline_cost += params.w_curv_avg * pow(abs_kappa / float(spline.kappaVector.size()), 2) * s_length;
        // peak curvature
        double max_min = abs(spline.kappaVector.array().maxCoeff() - spline.kappaVector.array().minCoeff());
        offline_cost += params.w_curv_peak * pow(max_min, 2) * s_length;

        // path length
        offline_cost += params.w_length * s_length;

        // raceline cost
        double raceline_dist = abs(nodeIndexesOnRaceline[end_layer] - end_node) * params.lat_resolution;
        double raceline_cost = min(params.w_raceline * s_length * raceline_dist, params.w_raceline_sat * s_length);

        offline_cost += raceline_cost;

        spline.cost = offline_cost;
        // cout << "(" << startPoint.first << ", " << startPoint.second << ") " << " -> " << "(" << end_layer << ", " << end_node << "): " << offline_cost << endl;
      }
    }
  }
  // -------------------------------------------
  // ---------------- filtering ----------------
  // -------------------------------------------

  void apply_node_filter(const std::vector<std::tuple<int,int,int>> &blocked_zones) {
      IPairAdjList &g = nodeGraph;

      for (auto &br : blocked_zones) {
          int layer, i, j;
          std::tie(layer, i, j) = br;

          for (int idx = i; idx <= j; ++idx) {
              IPair node = {layer, idx};

              // 이 노드와 연결된 edge들에 penalty 적용
              for (auto &dst : g[node]) {
                  double &edgeCost = splineMap[node][dst].cost;

                  // 원래 값을 기록해두기 (중복 기록 방지)
                  if (orig_edges.find({node, dst}) == orig_edges.end()) {
                      orig_edges[{node, dst}] = edgeCost;
                  }

                  // penalty 적용
                  edgeCost += 1e6;
              }
          }
      }
  }
// void apply_node_filter(
//     const std::vector<std::tuple<int,int,int>> &blocked_zones,
//     const f110_msgs::msg::Obstacle &target_obstacle,
//     NodeMap& nodeMap)
// {
//     IPairAdjList &g = nodeGraph;

//     // 장애물 기준 방향 (-1 = LEFT, +1 = RIGHT)
//     int obs_side = (target_obstacle.d_right > std::abs(target_obstacle.d_left)) ? 1 : -1;

//     for (auto &[layer, i, j] : blocked_zones) {

//         // zone 경계
//         int mid_idx = (i + j) / 2; // 중심 노드 기준으로 inner/outer 구분
//         int total_nodes = static_cast<int>(nodeMap[layer].size());

//         for (int idx = 0; idx < total_nodes; ++idx) {
//             IPair node = {layer, idx};

//             // zone 내부는 기존처럼 강하게 막기
//             if (idx >= i && idx <= j) {
//                 for (auto &dst : g[node]) {
//                     double &edgeCost = splineMap[node][dst].cost;
//                     if (orig_edges.find({node, dst}) == orig_edges.end()) {
//                         orig_edges[{node, dst}] = edgeCost;
//                     }
//                     edgeCost += 1e6;  // 하드 블록
//                 }
//                 continue;
//             }

//             // zone 밖 → inner / outer bias 적용
//             bool is_inner = (obs_side == 1) ? (idx < i) : (idx > j);
//             double side_factor = is_inner ? params.w_inner : params.w_outer;

//             for (auto &dst : g[node]) {
//                 double &edgeCost = splineMap[node][dst].cost;
//                 if (orig_edges.find({node, dst}) == orig_edges.end()) {
//                     orig_edges[{node, dst}] = edgeCost;
//                 }

//                 auto &spline = splineMap[node][dst];

//                 edgeCost = edgeCost * side_factor;
//             }
//         }
//     }
// }


  void deactivateFiltering() {
      // 수정했던 edge들만 복원
      for (auto &kv : orig_edges) {
          auto [src_dst, oldCost] = kv;
          auto src = src_dst.first;
          auto dst = src_dst.second;
          splineMap[src][dst].cost = oldCost;
      }

      orig_edges.clear(); // 기록 초기화
  }
  
  void hysteresisBias(const std::string& side, 
                      int cur_layer, 
                      IVector &nodeIndicesOnRaceline,
                      NodeMap &nodeMap,
                      int horizon_layers) {
      IPairAdjList &g = nodeGraph;

      for (int l = cur_layer; l < std::min(cur_layer + horizon_layers, num_layers); ++l) {
          int num_nodes = nodeMap[l].size();   // 레이어 내 노드 개수
          for (int idx = 0; idx < num_nodes; ++idx) {
              IPair node = {l, idx};

              bool penalize = false;
              if (side == "LEFT"  && idx > nodeIndicesOnRaceline[l]) penalize = true;
              if (side == "RIGHT" && idx < nodeIndicesOnRaceline[l]) penalize = true;

              if (penalize) {
                  for (auto &dst : g[node]) {
                      double &edgeCost = splineMap[node][dst].cost;
                      if (orig_edges.find({node, dst}) == orig_edges.end()) {
                          orig_edges[{node, dst}] = edgeCost;
                      }
                      edgeCost += 1e6;
                  }
              }
          }
      }

  }

  // blocked: 장애물이 가리는 노드 집합. nullptr 이 아니면 그 노드들은 탐색에서 아예 제외한다
  //   (= 하드 프루닝). 예전에는 apply_node_filter 로 나가는 edge 에 +1e6 벌점만 줬는데,
  //   벌점이 유한하므로 우회 경로의 곡률 비용이 1e6 을 넘는 순간 "장애물을 뚫고 가는 해"가
  //   최적이 되어버렸다. 그걸 runOnline 이 통째로 폐기해서 빈 경로가 나갔다(실측 73.6%).
  //   후보 생성 단계에서 빼버리면 그런 해가 애초에 만들어지지 않으므로,
  //   반환되는 경로는 항상 "장애물을 안 밟는 경로"이고, 못 찾으면 그건 진짜로 길이 없는 것이다.
  //   시작 노드(startIdx)는 예외다 - 차가 이미 차단 영역 안에 서 있을 수 있다.
  IPairVector graph_search(
    IPair startIdx,
    int goalLayer,
    const IVector& nodeIndicesOnRaceline,
    const rclcpp::Logger& logger,
    bool isClosedTrack = true,  // 트랙 폐회로 여부 인자 추가
    const std::set<IPair>* blocked = nullptr
) {
    std::unordered_map<IPair, double, pair_hash> dist;
    std::unordered_map<IPair, IPair, pair_hash> parent;
    std::priority_queue<std::pair<double, IPair>,
                        std::vector<std::pair<double, IPair>>,
                        std::greater<>> pq;

    dist[startIdx] = 0.0;
    pq.push({0.0, startIdx});

    size_t expanded = 0, pushed = 0, pruned = 0;
    IPair goalNode = {-1, -1};
    double bestCost = 1e12;

    const int totalLayers = static_cast<int>(nodeIndicesOnRaceline.size());
    const int startLayer = startIdx.first;

    // wrap-around aware helper lambda
    auto layerWithinRange = [&](int layer) -> bool {
        if (!isClosedTrack) {
            return layer <= goalLayer;
        } else {
            // 폐회로일 때는 goalLayer가 start보다 작을 수도 있음
            if (startLayer <= goalLayer) {
                return layer <= goalLayer;
            } else {
                // wrap-around 구간: start=75, goal=10 같은 케이스
                return (layer >= startLayer || layer <= goalLayer);
            }
        }
    };

    while (!pq.empty()) {
        auto [cost, u] = pq.top();
        pq.pop();
        expanded++;

        // goal layer 도달
        if (u.first == goalLayer) {
            if (cost < bestCost) {
                bestCost = cost;
                goalNode = u;
            }
            continue;
        }

        for (auto &v : getChildList(u)) {
            if (!layerWithinRange(v.first)) continue;
            // 하드 프루닝: 차단 노드는 후보에 올리지 않는다 (비용 비교 자체를 없앤다)
            if (blocked && blocked->count(v)) { pruned++; continue; }

            double edgeCost = splineMap[u][v].cost;
            double newCost  = cost + edgeCost;

            if (dist.find(v) == dist.end() || dist[v] > newCost) {
                dist[v] = newCost;
                parent[v] = u;
                pq.push({newCost, v});
                pushed++;
            }
        }
    }

    // fallback: goalLayer 확장
    int layerOffset = 1;
    while (goalNode.first == -1 && layerOffset <= 2) {
        int altLayer = (goalLayer + layerOffset) % totalLayers;
        IPair altGoal = {altLayer, nodeIndicesOnRaceline[altLayer]};
        if (dist.find(altGoal) != dist.end()) {
            goalNode = altGoal;
            bestCost = dist[altGoal];
            // RCLCPP_WARN(logger,
            //             "[fallback] Using layer %d raceline node (%d,%d) with cost %.2f",
            //             altLayer, altGoal.first, altGoal.second, bestCost);
            break;
        }
        layerOffset++;
    }

    if (goalNode.first == -1) {
        // 하드 프루닝이면 통로가 막힌 동안 매 주기 찍히므로 1 Hz 로 조인다.
        static rclcpp::Clock s_throttle_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(logger, s_throttle_clock, 1000,
                    "graph_search(): No valid goal found up to layer %d "
                    "(expanded=%zu, pushed=%zu, pruned=%zu) -> 자유 통로가 실제로 없다",
                    goalLayer, expanded, pushed, pruned);
        return {};
    }

    // RCLCPP_INFO(logger,
    //             "Selected goal node: layer=%d, node=%d, cost=%.2f (expanded=%zu, pushed=%zu)",
    //             goalNode.first, goalNode.second, bestCost, expanded, pushed);

    // backtrack
    IPairVector path;
    IPair cur = goalNode;
    while (cur != startIdx) {
        path.push_back(cur);
        cur = parent[cur];
    }
    path.push_back(startIdx);
    std::reverse(path.begin(), path.end());

    return path;
}


};