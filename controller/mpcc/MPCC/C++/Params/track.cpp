// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "track.h"
namespace mpcc{
Track::Track(std::string file) 
{   
    int scale = 1; //0518 추가

    /////////////////////////////////////////////////////
    // Loading Model and Constraint Parameters //////////
    /////////////////////////////////////////////////////
    std::ifstream iTrack(file);
    json jsonTrack;
    iTrack >> jsonTrack;
    // Model Parameters
    // Eigen::Map()은 외부 데이터(ex. std::vector)를 Eigen 객체로 감싸는 래퍼(wrapper) : 값의 복사 없이 기존 데이터를 Eigen 타입처럼 사용할 수 있게 해줌. 즉, 여기서는 std::vector -> Eigen::VectorXd
    // x.data(): std::vector의 포인터(double*)를 반환
    // x.size(): 배열의 길이
    std::vector<double> x = jsonTrack["X"];
    X = scale * Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
    std::vector<double> y = jsonTrack["Y"];
    Y = scale * Eigen::Map<Eigen::VectorXd>(y.data(), y.size());
    
    std::vector<double> x_inner = jsonTrack["X_i"];
    X_inner = scale * Eigen::Map<Eigen::VectorXd>(x_inner.data(), x_inner.size());
    std::vector<double> y_inner = jsonTrack["Y_i"];
    Y_inner = scale * Eigen::Map<Eigen::VectorXd>(y_inner.data(), y_inner.size());

    std::vector<double> x_outer = jsonTrack["X_o"];
    X_outer = scale * Eigen::Map<Eigen::VectorXd>(x_outer.data(), x_outer.size());
    std::vector<double> y_outer = jsonTrack["Y_o"];
    Y_outer = scale * Eigen::Map<Eigen::VectorXd>(y_outer.data(), y_outer.size());
}

//track 정보를  TrackPos라는 구조체로 return해줌
TrackPos Track::getTrack()
{
    return {Eigen::Map<Eigen::VectorXd>(X.data(), X.size()), Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size()),
            Eigen::Map<Eigen::VectorXd>(X_inner.data(), X_inner.size()), Eigen::Map<Eigen::VectorXd>(Y_inner.data(), Y_inner.size()),
            Eigen::Map<Eigen::VectorXd>(X_outer.data(), X_outer.size()), Eigen::Map<Eigen::VectorXd>(Y_outer.data(), Y_outer.size())};
}
}
