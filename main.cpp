#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <Integrator.h>
#include <RCS.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using nlohmann::json;

int squareWave(double time, double period) {
    double currentTime = std::fmod(time, period);
    int amplitude;
    if (currentTime < (period / 4))  {
        amplitude = 1;
    }
    else if (currentTime < (2*period / 4)) {
        amplitude = 0;
    }
    else if (currentTime < (3*period / 4)) {
        amplitude = -1;
    }
    else {
        amplitude = 0;
    }
    return amplitude;
}

int constActiveSignal(double time) {
    return 1;
}

int constInactiveSignal(double time) {
    return 0;
}

std::vector<int> constActiveController(double time, VectorXd stateVector) {
    return std::vector<int> {constActiveSignal(time), constActiveSignal(time), constActiveSignal(time)};
}

std::vector<int> constInactiveController(double time, VectorXd stateVector) {
    return std::vector<int> {constInactiveSignal(time), constInactiveSignal(time), constInactiveSignal(time)};
}

std::vector<int> squareWaveController(double time, VectorXd stateVector) {
    return std::vector<int> {squareWave(time, 10), squareWave(time, 20), squareWave(time, 30)};
}

std::function<std::vector<int>(double, VectorXd)> getController(std::string const &controllerChoice) {
    if (controllerChoice == "constActive") {
        return constActiveController;
    }
    if (controllerChoice == "constInactive") {
        return constInactiveController;
    }
    if (controllerChoice == "squareWaveController") {
        return squareWaveController;
    }
    throw std::runtime_error("Unavailable controller selected: " + controllerChoice);
}

class Satellite{
    public:
        double mass;
        Eigen::Matrix3d moi = MatrixXd(3, 3);

        Eigen::Vector3d position = VectorXd(3);
        Eigen::Vector3d velocity = VectorXd(3);
        Eigen::Vector3d acceleration = VectorXd(3);
        Eigen::Vector3d force = VectorXd(3);
        Eigen::Quaterniond angularPosition; //w, x, y, z
        Eigen::Vector3d angularVelocity = VectorXd(3);
        Eigen::Vector3d angularAcceleration = VectorXd(3);
        Eigen::Vector3d moments = VectorXd(3);
        std::vector<Thruster> thrusters;
        std::function<std::vector<int>(double, VectorXd)> controller;
        std::vector<int> controlSignals;

        Satellite(json config);
        Eigen::Vector3d getBodyForce(double time);
        Eigen::Vector3d getBodyMoment(double time);
        Eigen::Vector3d getBodyForce(double time, VectorXd stateVector);  // Uses supplied state vector, instead of internal
        Eigen::Vector3d getBodyMoment(double time, VectorXd stateVector); // Uses supplied state vector, instead of internal
        VectorXd getStateVector(); //  [pos_1, pos_2, pos_3, vel_1, vel_2, vel_3, q.x(), q.y(), q.z(), q.w(), w_1, w_2, w_3]
        void setStateVector(VectorXd stateVector); //  [pos_1, pos_2, pos_3, vel_1, vel_2, vel_3, q.x(), q.y(), q.z(), q.w(), w_1, w_2, w_3]
        void setControlState(double time);
        VectorXd getStateDerivative(double time);
        VectorXd getStateDerivative(double time, VectorXd stateVector);
        Eigen::Vector3d getEulerAngles(int third, int second, int first); //C(third) <- C(second) <- C(first) Euler angles
        Eigen::Vector3d getEulerAngles321();
        Eigen::Matrix3d getRotationMatrix();
        Eigen::Vector3d getAngularMomentum();
        double getRotationalKineticEnergy();
        void setEulerAngles(VectorXd eulerAngles);
};

Satellite::Satellite(json config) {
    position << config["initialPosition"][0], config["initialPosition"][1], config["initialPosition"][2];
    velocity << config["initialVelocity"][0], config["initialVelocity"][1], config["initialVelocity"][2];
    acceleration << config["initialAcceleration"][0], config["initialAcceleration"][1], config["initialAcceleration"][2];
    angularPosition = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    angularVelocity << config["initialAngularVelocity"][0], config["initialAngularVelocity"][1], config["initialAngularVelocity"][2];
    angularAcceleration << config["initialAngularVelocity"][0], config["initialAngularVelocity"][1], config["initialAngularVelocity"][2];

    moi << config["moi"][0][0], config["moi"][0][1], config["moi"][0][2],
            config["moi"][1][0], config["moi"][1][1], config["moi"][1][2], 
            config["moi"][2][0], config["moi"][2][1], config["moi"][2][2];
    mass = config["mass"];

    for (int i = 0; i < config["thrusters"].size(); i++){
        auto currentThruster = config["thrusters"][i];
        Eigen::Vector3d controlVector;
        controlVector << currentThruster["controlVector"][0], currentThruster["controlVector"][1], currentThruster["controlVector"][2];
        thrusters.push_back(Thruster(controlVector, currentThruster["moment"]));
    }
    controlSignals = {0, 0, 0};
    controller = getController(config["controller"]);
}

Eigen::Vector3d Satellite::getBodyForce(double time) {
    return getBodyForce(time, getStateVector());
}

Eigen::Vector3d Satellite::getBodyForce(double time, VectorXd stateVector) {
    force << 0, 0, 0;
    return force;
}

Eigen::Vector3d Satellite::getBodyMoment(double time) {
    return getBodyMoment(time, getStateVector());
}

Eigen::Vector3d Satellite::getBodyMoment(double time, VectorXd stateVector) {
    Eigen::Vector3d moments = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < thrusters.size(); i++){
        moments += thrusters[i].getBodyMoment(time);
    }
    return moments;
}

void Satellite::setStateVector(VectorXd stateVector) {
    position << stateVector(0), stateVector(1), stateVector(2);
    velocity << stateVector(3), stateVector(4), stateVector(5);
    angularPosition = Eigen::Quaterniond(stateVector(9), stateVector(6), stateVector(7), stateVector(8)); // w, x, y, z
    angularPosition = angularPosition.normalized();
    angularVelocity << stateVector(10), stateVector(11), stateVector(12);
}

VectorXd Satellite::getStateVector() {
    VectorXd state_vector = VectorXd(13);
    state_vector << position(0), position(1), position(2), velocity(0), velocity(1), velocity(2),
                    angularPosition.x(), angularPosition.y(), angularPosition.z(), angularPosition.w(),
                    angularVelocity(0), angularVelocity(1), angularVelocity(2);
    return state_vector;
}

VectorXd Satellite::getStateDerivative(double time) {
    return getStateDerivative(time, getStateVector());
}

VectorXd Satellite::getStateDerivative(double time, VectorXd stateVector) {
    Eigen::Vector3d velocity(stateVector(3), stateVector(4), stateVector(5));
    Eigen::Vector3d angularVelocity(stateVector(10), stateVector(11), stateVector(12)); 

    force = getBodyForce(time, stateVector);
    moments = getBodyMoment(time, stateVector);

    acceleration = force / mass;
    angularAcceleration = moi.colPivHouseholderQr().solve(moments - angularVelocity.cross(moi * angularVelocity));
    // angularAcceleration = moi.colPivHouseholderQr().solve(moments);

    Eigen::Matrix4d angularRateMatrix;
    angularRateMatrix << 0, angularVelocity(2), -angularVelocity(1), angularVelocity(0),
                        -angularVelocity(2), 0, angularVelocity(0), angularVelocity(1),
                        angularVelocity(1), -angularVelocity(0), 0, angularVelocity(2),
                        -angularVelocity(0), -angularVelocity(1), -angularVelocity(2), 0;

    Eigen::Vector4d quaternionVector = angularPosition.coeffs(); // x, y, z, w
    Eigen::Vector4d quaternionDot = 0.5 * angularRateMatrix * quaternionVector;

    VectorXd derivative = VectorXd(13);
    derivative << velocity(0), velocity(1), velocity(2), 
                  acceleration(0), acceleration(1), acceleration(2), 
                  quaternionDot(0), quaternionDot(1), quaternionDot(2), quaternionDot(3), 
                  angularAcceleration(0), angularAcceleration(1), angularAcceleration(2);
    
    return derivative;
}

void Satellite::setControlState(double time) {
    controlSignals = controller(time, getStateVector());
    for (int i = 0; i < thrusters.size(); i++){
        thrusters[i].setControlState(time, controlSignals[i]);
    }
}

Eigen::Vector3d Satellite::getEulerAngles(int third, int second, int first) {
    return getRotationMatrix().eulerAngles(third, second, first); // For args a, b, c, returns the transform defined by C(theta_a) <- C(theta_b) <- C(theta_c)
}

Eigen::Vector3d Satellite::getEulerAngles321() {
    // Gets the Euler angles that correspond to Bong's C(th_3) <- C(th_2) < C(th_1) Euler angles
    auto rotation_matrix = getRotationMatrix().transpose();
    auto theta_1 = std::atan2(-rotation_matrix(2, 1), rotation_matrix(2, 2));
    auto theta_2 = std::asin(rotation_matrix(2, 0));
    auto theta_3 = std::atan2(-rotation_matrix(1, 0), rotation_matrix(0, 0));
    Eigen::Vector3d eulers(theta_1, theta_2, theta_3);
    return eulers;
}

Eigen::Matrix3d Satellite::getRotationMatrix() {
    return angularPosition.toRotationMatrix();
}

Eigen::Vector3d Satellite::getAngularMomentum() {
    return moi * angularVelocity;
}

double Satellite::getRotationalKineticEnergy() {
    return 0.5 * angularVelocity.transpose() * moi * angularVelocity;
}

class Simulation{
    public:
        double t_end;
        double t;
        double dt;
        json config;
        Satellite sat;

        static json readConfigFile(const std::string configString);
        Simulation(const std::string configString);
        void logState(std::ofstream &);
        void runMainLoop(std::ofstream &);
    private:
        bool first_log = false;
};

json Simulation::readConfigFile(const std::string configString) {
    std::ifstream config(configString);
    std::stringstream buffer;
    buffer << config.rdbuf();
    return json::parse(buffer.str());
}

Simulation::Simulation(const std::string configString) : config(readConfigFile(configString)), sat(config)  {
    t_end = config["tEnd"];
    t = 0;
    dt = config["dt"];
}

void Simulation::logState(std::ofstream &file) {
    if (!first_log) {
        file << "time";
        file << "," << "s_1" << "," << "s_2" << "," << "s_3";
        file << "," << "v_1" << "," << "v_2" << "," << "v_3";
        file << "," << "a_1" << "," << "a_2" << "," << "a_3";
        file << "," << "theta_1" << "," << "theta_2" << "," << "theta_3";
        file << "," << "quaternion_1" << "," << "quaternion_2" << "," << "quaternion_3" << "," << "quaternion_4";
        file << "," << "dcm_1" << "," << "dcm_2" << "," << "dcm_3" << "," << "dcm_4" << "," << "dcm_5" << "," << "dcm_6" << "," << "dcm_7" << "," << "dcm_8" << "," << "dcm_9";
        file << "," << "w_1" << "," << "w_2" << "," << "w_3";
        file << "," << "alpha_1" << "," << "alpha_2" << "," << "alpha_3";
        file << "," << "force_1" << "," << "force_2" << "," << "force_3";
        file << "," << "moments_1" << "," << "moments_2" << "," << "moments_3";
        file << "," << "control_signal_1" << "," << "control_signal_2" << "," << "control_signal_3";
        file << "," << "angular_momentum";
        file << "," << "rotational_kinetic_energy";
        file << std::endl;
        first_log = true;
    }
    auto eulerAngles = sat.getEulerAngles321();
    auto dcm = sat.getRotationMatrix();
    file << t;
    file << "," << sat.position(0) << "," << sat.position(1) << "," << sat.position(2);
    file << "," << sat.velocity(0) << "," << sat.velocity(1) << "," << sat.velocity(2);
    file << "," << sat.acceleration(0) << "," << sat.acceleration(1) << "," << sat.acceleration(2);
    file << "," << eulerAngles(0) << "," << eulerAngles(1) << "," << eulerAngles(2);
    file << "," << sat.angularPosition.x() << "," << sat.angularPosition.y() << "," << sat.angularPosition.z() << "," << sat.angularPosition.w(); 
    file << "," << dcm(0, 0) << "," << dcm(0, 1) << "," << dcm(0, 2) << "," << dcm(1, 0) << "," << dcm(1, 1) << "," << dcm(1, 2) << "," << dcm(2, 0) << "," << dcm(2, 1) << "," << dcm(2, 2);
    file << "," << sat.angularVelocity(0) << "," << sat.angularVelocity(1) << "," << sat.angularVelocity(2);
    file << "," << sat.angularAcceleration(0) << "," << sat.angularAcceleration(1) << "," << sat.angularAcceleration(2);
    file << "," << sat.force(0) << "," << sat.force(1) << "," << sat.force(2);
    file << "," << sat.moments(0) << "," << sat.moments(1) << "," << sat.moments(2);
    file << "," << sat.controlSignals[0] << "," << sat.controlSignals[1] << "," << sat.controlSignals[2];
    file << "," << sat.getAngularMomentum().norm();
    file << "," << sat.getRotationalKineticEnergy();
    file << std::endl;
}

void Simulation::runMainLoop(std::ofstream &file) {
    logState(file);
    VectorXd new_state;

    while (t < t_end) {
        sat.setControlState(t);
        std::tie(t, new_state, dt) = integrate_Rkf([this](double t, VectorXd y) {return sat.getStateDerivative(t, y);}, t, sat.getStateVector(), dt);
        sat.setStateVector(new_state);
        logState(file);
    }
}

int main() { 
    Simulation sim(std::string("Ch_7.1.2.json"));
    std::ofstream myFile("data.csv");
    sim.runMainLoop(myFile);
    myFile.close();
    return 0;

}