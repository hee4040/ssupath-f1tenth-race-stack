#pragma once
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>  
#include <cmath>
#include <algorithm>
#include <time.h>
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <iterator>
#include <functional>
#include <omp.h>
#include <yaml-cpp/yaml.h>
#include "matplotlibcpp.h"
#include "f110_msgs/msg/obstacle.hpp" 

#include <Eigen/Dense>
#include "rapidcsv.h"
//////////////////////////////////////////////////////////////////////
// Index names used in gtplMap
// #define INDEX_NAME  "COLUME_NAME_in_TRAJ_LTPL_CSV_FILE"
//////////////////////////////////////////////////////////////////////

// Index names originated from TRAJ_LTPL_CSV_FILE
#define POS_X       "x_ref_m"
#define POS_Y       "y_ref_m" // Do not delete the first white space in the following!!!
#define WIDTH_L     "width_left_m"
#define WIDTH_R     "width_right_m"
#define NORM_X      "x_normvec_m"
#define NORM_Y      "y_normvec_m"
#define NORM_L      "alpha_m"
#define RL_KAPPA    "kappa_racetraj_radpm"
#define RL_S        "s_racetraj_m"
#define RL_PSI      "psi_racetraj_rad"
#define RL_VX       "vx_racetraj_mps"

// Index names additionally created for this program
#define RL_dS       "delta_s"
#define RL_X        "x_raceline"
#define RL_Y        "y_raceline"
#define LB_X        "x_bound_l"
#define LB_Y        "y_bound_l"
#define RB_X        "x_bound_r"
#define RB_Y        "y_bound_r"
#define LB_PSI      "psi_bound_l"
#define RB_PSI      "psi_bound_r"

using namespace std;
using namespace rapidcsv;
using namespace Eigen;
namespace plt = matplotlibcpp;

struct Node {
    double x;
    double y;
    double psi;
    bool raceline;
};

struct SplineInfo {
    MatrixXd coeffs_x;          
    MatrixXd coeffs_y;          
    VectorXd kappaVector;
    VectorXd el_lengths;
    vector<Vector2d> points_xy;
    double cost;
    bool raceline;
};

struct SplineSample {
    double x, y;
    double x_d, y_d;
    double x_dd, y_dd;
    double kappa;
};

typedef vector<double> DVector;
typedef vector<int>    IVector;
typedef map<string, DVector> DMap;
typedef map<string, IVector> IMap;
typedef vector<vector<Node>> NodeMap;

typedef pair<int, int> IPair; // <layerIdx, nodeIdx>
typedef vector<IPair> IPairVector; // 엣지 연결 여부 확인용 value vector
typedef map<IPair, IPairVector> IPairAdjList; // key: 기준 노드, value: key와 연결된 다음 레이어의 노드 인덱스 IPair
typedef map<IPair, map<IPair, SplineInfo>> SplineMap;

// // utilities.cpp
double normalizeAngle(double angle);
void map_size(DMap &map);
unique_ptr<string> Load(const string &filename);
DMap readDMapFromCSV(const string &pathname);
void writeDMapToCSV(const string &pathname, DMap &map, char delimiter = ',');
void visualizeTrajectories(DMap &gtMap, DMap &stMap, const NodeMap &nodeMap, SplineMap &splineMap);
