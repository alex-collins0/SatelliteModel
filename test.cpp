#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using nlohmann::json;

int main() {
    // Eigen::Matrix3d C1;
    // C1 = Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX());
    // Eigen::Matrix3d C2;
    // C2 = Eigen::AngleAxisd(2, Eigen::Vector3d::UnitY());
    // Eigen::Matrix3d C3;
    // C3 = Eigen::AngleAxisd(3, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d mat = C3 * C2 * C1;
    // std::cout << C1 << std::endl;
    // std::cout << C2 << std::endl;
    // std::cout << C3 << std::endl;
    // std::cout << mat << std::endl;
    // std::cout << mat.eulerAngles(2, 1, 0) << std::endl;
    
    std::ifstream config("config.json");
    std::stringstream buffer;
    buffer << config.rdbuf();
    json j(json::parse(buffer.str()));

    std::cout << j["t_end"] << std::endl;
    std::cout << j["moi"][2][2] << std::endl;
    std::cout << typeid(j["moi"]).name() << std::endl;

    for (int i = 0; i < j["thrusters"].size(); i++){
        std::cout << j["thrusters"][i] << std::endl;
    }
    std::cout << typeid("test").name();
}