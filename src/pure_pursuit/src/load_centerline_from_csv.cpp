#include "pure_pursuit/load_centerline_from_csv.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

std::vector<geometry_msgs::Point> loadCenterlineFromCSV(const std::string& filename) {
    std::vector<geometry_msgs::Point> centerline;
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        // 첫 번째 줄은 헤더이므로 무시합니다.
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string x_str, y_str;
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                geometry_msgs::Point point;
                point.x = std::stod(x_str);
                point.y = std::stod(y_str);
                point.z = 0.0; // 필요에 따라 z 값 설정
                centerline.push_back(point);
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }

    return centerline;
}

