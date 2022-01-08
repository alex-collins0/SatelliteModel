#ifndef RCS_H
#define RCS_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class ThrusterState;

class Thruster {
    public:
        Eigen::Vector3d controlVector;
        double controlMoment, riseTime, fallTime;
        
        Thruster(Eigen::Vector3d currentControlVector, double givenControlMoment);
        void setControlState(double time, int controlSignal);
        Eigen::Vector3d getBodyMoment(double time);

    private:
        ThrusterState state;
        double timeLastControlSignalChange;
        double timeOffset; // This is used to account for the fact we might change state during rising/falling, and we want to capture how the reduction in rise/fall time due to an intermediate start point
        void setThrusterInternalState(double time, ThrusterState givenState);
        double getEffectiveTime(double time);
        int controlSignal;
        int controlDirection; // Stores whether we're firing the RCS backwards or forwards
};

#endif