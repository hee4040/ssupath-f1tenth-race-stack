#include "graph_planner.hpp"

void plotHeading(const DVector &x,
                 const DVector &y,
                 const DVector &psi,
                 double scale = 0.5)
{
    double dx, dy;
    double theta, arrow_len;
    double angle;
    double x_arrow1, y_arrow1;
    double x_arrow2, y_arrow2;

    for (size_t i = 0; i < x.size(); ++i) {
        dx = scale * cos(psi[i] + M_PI_2);
        dy = scale * sin(psi[i] + M_PI_2);

        // psi 방향 
        DVector x_line = {x[i], x[i] + dx};
        DVector y_line = {y[i], y[i] + dy};
        // plt::plot(x_line, y_line, {{"color", "green"}});

        #if 0
        // 화살촉 
        theta = atan2(dy, dx);
        arrow_len = 0.2 * scale;
        angle = M_PI / 6.0;  // 30 degrees

        x_arrow1 = x[i] + dx - arrow_len * cos(theta - angle);
        y_arrow1 = y[i] + dy - arrow_len * sin(theta - angle);

        x_arrow2 = x[i] + dx - arrow_len * cos(theta + angle);
        y_arrow2 = y[i] + dy - arrow_len * sin(theta + angle);

        // 화살촉 그리기 
        plt::plot({x[i] + dx, x_arrow1}, {y[i] + dy, y_arrow1}, {{"color", "green"}});
        plt::plot({x[i] + dx, x_arrow2}, {y[i] + dy, y_arrow2}, {{"color", "green"}});
        #endif

    }
        // raceline 좌표와 psi 프린팅 
        #if 1
        for (size_t i = 0; i < x.size(); ++i) {
        ostringstream label;
        label.precision(2);
        label << fixed << "(" << x[i] << ", " << y[i] << ")\nψ=" << psi[i];

        plt::text(x[i], y[i], label.str());
    }
        #endif
}

void plotHeading(const NodeMap& nodeMap, double scale = 0.5) {
    DVector x_line, y_line;
    DVector node_x, node_y;
    int layer_idx, node_idx;
    for (const auto& layer_nodes : nodeMap) {
        for (const auto& node : layer_nodes) {
            double dx = scale * cos(node.psi + M_PI_2);
            double dy = scale * sin(node.psi + M_PI_2);

            node_x.push_back(node.x);
            node_y.push_back(node.y);
            plt::scatter(node_x, node_y, 10.0, {{"color", "purple"}});

            ostringstream label;
            label.precision(2);
            label << fixed << layer_idx << ", " << node_idx;

            plt::text(layer_idx, node_idx, label.str());
            

            #if 0
            x_line = {node.x, node.x + dx};
            y_line = {node.y, node.y + dy};
            plt::plot(x_line, y_line, {{"color", "purple"}});

            // 화살촉 (arrowhead)
            
            double theta = atan2(dy, dx);
            double arrow_len = 0.2 * scale;
            double angle = M_PI / 6.0;

            double x_arrow1 = node.x + dx - arrow_len * cos(theta - angle);
            double y_arrow1 = node.y + dy - arrow_len * sin(theta - angle);

            double x_arrow2 = node.x + dx - arrow_len * cos(theta + angle);
            double y_arrow2 = node.y + dy - arrow_len * sin(theta + angle);

            plt::plot({node.x + dx, x_arrow1}, {node.y + dy, y_arrow1}, {{"color", "purple"}});
            plt::plot({node.x + dx, x_arrow2}, {node.y + dy, y_arrow2}, {{"color", "purple"}});
            #endif
            }
            layer_idx++;
    }
}

void plotAllSplines(SplineMap& splineMap, const NodeMap& nodeMap, const string &color) {
    unordered_set<int> plotted_layers;

    for (auto& [startPoint, endPoints] : splineMap) {
        for (auto& [endPoint, spline] : endPoints) {

            const Node& startNode = nodeMap[startPoint.first][startPoint.second];
            const Node& endNode = nodeMap[endPoint.first][endPoint.second];

            DVector xs, ys;
            for (auto& pt : spline.points_xy) {
                xs.push_back(pt.x());
                ys.push_back(pt.y());
            }

            // 선 그리기
            plt::plot(xs, ys, {{"color", color}});

            // 레이어 Text 표시
            if (plotted_layers.find(startPoint.first) == plotted_layers.end() && !spline.points_xy.empty()) {
                const auto& pos_pt = spline.points_xy[spline.points_xy.size() / 8];
                string layer_label = "L" + to_string(startPoint.first);
                plt::text(pos_pt.x(), pos_pt.y(), layer_label);
                plotted_layers.insert(startPoint.first);
            }
        }
    }
}


void plotSpline(const SplineInfo& spline, const string& color) {

    if (spline.points_xy.empty()) {
        // cerr << "Warning: spline.points_xy is empty! Nothing to plot." << endl;
        return;
    }

    DVector xs, ys;
    for (const auto& pt : spline.points_xy) {
        xs.push_back(pt.x());
        ys.push_back(pt.y());
    }

    plt::plot(xs, ys, {{"color", color}, {"linewidth", "4.0"}});

    // plt::title("Spline Path");
    // plt::xlabel("X");
    // plt::ylabel("Y");
    // plt::axis("equal");
    // plt::show();
}

void visualizeTrajectories(DMap &gtMap,
            DMap &stMap,
            const NodeMap &nodeMap,
            SplineMap &splineMap) {

    plt::plot(gtMap[LB_X], gtMap[LB_Y], {{"color", "orange"}});
    plt::plot(gtMap[RB_X], gtMap[RB_Y], {{"color", "orange"}});
    
    // plt::plot(gtMap[x_ref], gtMap[y_ref], {{"color", "blue"}});
    plt::plot(gtMap[RL_X], gtMap[RL_Y], {{"color", "red"}});

    plt::scatter(stMap[RL_X], stMap[RL_Y], 30.0, {{"color", "red"}});

    // plotHeading(stMap[x_raceline],
    //             stMap[y_raceline],
    //             stMap[__psi]);

    // plotHeading(stMap[x_bound_l],
    //             stMap[y_bound_l],
    //             psi_bound_l);

    // plotHeading(stMap[x_bound_r],
    //             stMap[y_bound_r],
    //             psi_bound_r);

    // 노드마다 psi확인할 수 있는 용도 
    plotHeading(nodeMap);

    plotAllSplines(splineMap, nodeMap, "gray");

    plt::title("Track");
    plt::grid(true);
    plt::axis("equal");
    plt::show();
 
}