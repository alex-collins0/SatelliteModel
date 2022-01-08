#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <RCS.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class ThrusterState {
    INACTIVE = 0,
    RISING = 1,
    ACTIVE = 2,
    FALLING = 3
};

Thruster::Thruster(Eigen::Vector3d givenControlVector, double givenControlMoment) {
    controlVector = givenControlVector;
    controlMoment = givenControlMoment;
    riseTime = 0.1;
    fallTime = 0.1;
    state = ThrusterState::INACTIVE;
    timeLastControlSignalChange = 0;
    timeOffset = 0;
    controlSignal = 0;
    controlDirection = 1;
}

double Thruster::getEffectiveTime(double time) {
    return time + timeOffset;
}

void Thruster::setControlState(double time, int currentControlSignal) {
    double intendedMoment;
    controlSignal = currentControlSignal;
    if (currentControlSignal == 1 && controlDirection == -1) {
        controlDirection = currentControlSignal;
    }
    else if (currentControlSignal == -1 && controlDirection == 1) { 
        controlDirection = currentControlSignal;
    }
    if (controlSignal) {
        switch (state) {
            case ThrusterState::INACTIVE:
                setThrusterInternalState(time, ThrusterState::RISING);
                break;

            case ThrusterState::RISING:
                if ((getEffectiveTime(time) - timeLastControlSignalChange) > riseTime) {
                    setThrusterInternalState(time, ThrusterState::ACTIVE); 
                }
                break;

            case ThrusterState::ACTIVE:
                break;

            case ThrusterState::FALLING: 
                // Calculate how much force we would be outputting
                intendedMoment = controlMoment * (getEffectiveTime(time) - timeLastControlSignalChange) / fallTime;
                timeOffset = riseTime * intendedMoment / controlMoment; // This is how much further ahead we'd have to be to be at the same force
                setThrusterInternalState(time, ThrusterState::RISING);
                break;
        }

    }
    else {
        switch (state) {
            case ThrusterState::INACTIVE:
                break;

            case ThrusterState::RISING:
                // Calculate how much force we would be outputting
                intendedMoment = controlMoment * (getEffectiveTime(time) - timeLastControlSignalChange) / riseTime;
                timeOffset = fallTime * intendedMoment / controlMoment; // This is how much further ahead we'd have to be to be at the same force
                setThrusterInternalState(time, ThrusterState::FALLING);
                break;

            case ThrusterState::ACTIVE:
                setThrusterInternalState(time, ThrusterState::FALLING);
                break;

            case ThrusterState::FALLING: 
                if ((getEffectiveTime(time) - timeLastControlSignalChange) > fallTime) {
                    setThrusterInternalState(time, ThrusterState::INACTIVE);
                }
                break;
        }
    }
}

void Thruster::setThrusterInternalState(double time, ThrusterState givenState) {
    timeLastControlSignalChange = time;
    state = givenState;
}

Eigen::Vector3d Thruster::getBodyMoment(double time) {
    double moment = 0;
    switch (state) {
        case ThrusterState::INACTIVE:
            moment = 0;
            break;
        case ThrusterState::RISING:
            moment = controlMoment * (getEffectiveTime(time) - timeLastControlSignalChange) / riseTime;
            break;
        case ThrusterState::ACTIVE:
            moment = controlMoment;
            break;
        case ThrusterState::FALLING:
            moment = controlMoment * (1 - (getEffectiveTime(time) - timeLastControlSignalChange) / fallTime);
            break;
    }
    VectorXd controlMomentVector = moment * controlDirection * controlVector;
    return controlMomentVector;
      
}
