#include "graph_planner.hpp"

unique_ptr<string> Load(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Could not open INI file: " << filename << endl;
    }

    string line;
    bool in_section = false;
    while (getline(file, line)) {
        // line이라는 문자열에서 해당 문자열을 찾지 못하면 npos를 반환
        if (line.find("[DRIVING_TASK]") != string::npos) {
            in_section = true;
            continue;
        }

        if (in_section && line.find('[') != string::npos)
            break;

        // track 키 찾기
        if (in_section && line.find("track") != string::npos) {
            size_t eq_pos = line.find('=');
            if (eq_pos != string::npos) {
                string value = line.substr(eq_pos + 1);
                value.erase(0, value.find_first_not_of(" \t\r\n"));
                value.erase(value.find_last_not_of(" \t\r\n") + 1);
                return make_unique<string>(value);
            }
        }
    }
    throw logic_error("Unreachable exit of the while loop!");
}

double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

//////////////////////////////////////////////////////////////////////////////
// DMap dependent functions
//////////////////////////////////////////////////////////////////////////////

// Debug용 함수: map의 columns, rows 개수 print  
void map_size(DMap& map) {
    size_t num_cols = map.size();
    size_t num_rows = map.begin()->second.size();
    cout << "mapsize(" << num_rows << "," << num_cols << ")" << endl;
}

// CSV를 읽어서 DMap으로 변경 
DMap readDMapFromCSV(const string& pathname) {
    DMap gtMap;
    Document csv(pathname, LabelParams(0, -1), SeparatorParams(';'));
    vector<string> labels = csv.GetColumnNames();

    for (const auto& label : labels) 
        gtMap[label] = csv.GetColumn<double>(label);

    return gtMap;
}

// DMap을 CSV에 작성 
void writeDMapToCSV(const string& pathname, DMap& map, char delimiter) {
    ofstream file(pathname);
    if (!file.is_open()) throw runtime_error("Can't open file.");

    size_t num_cols = map.size();
    size_t num_rows = map.begin()->second.size();

    // Header
    size_t i = 0;
    for (const auto& [key, _] : map) {
        file << key;
        if (++i != num_cols) file << delimiter;
    }
    file << '\n';

    // Row map
    for (size_t row = 0; row < num_rows; ++row) {
        size_t j = 0;
        for (const auto& [_, col] : map) {
            file << col[row];
            if (++j != num_cols) file << delimiter;
        }
        file << '\n';
    }

    file.close();
}

void printSplineInfo(const SplineMap& splineMap, const NodeMap& nodeMap) {

    for (auto& [startPoint, endPoints] : splineMap) {
        for (auto& [endPoint, spline] : endPoints) {


        const Node& startNode = nodeMap[startPoint.first][startPoint.second];
        const Node& endNode = nodeMap[endPoint.first][endPoint.second];

        cout << "\n(" << startPoint.first << ", " << startPoint.second << ") --> ("
                  << endPoint.first << ", " << endPoint.second << ")\n";

        cout << "  [Start Node] x: " << startNode.x
                  << ", y: " << startNode.y
                  << ", psi: " << startNode.psi << "\n";
        cout << "  [End Node]   x: " << endNode.x
                  << ", y: " << endNode.y
                  << ", psi: " << endNode.psi << "\n";

        cout << "  coeffs_x (" << spline.coeffs_x.rows() << "x" << spline.coeffs_x.cols() << "):\n";
        cout << spline.coeffs_x << "\n";

        cout << "  coeffs_y (" << spline.coeffs_y.rows() << "x" << spline.coeffs_y.cols() << "):\n";
        cout << spline.coeffs_y << "\n";

        cout << "----------------------------------------";
        }
    }
}