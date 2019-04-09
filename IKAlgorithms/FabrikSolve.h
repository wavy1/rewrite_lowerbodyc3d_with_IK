//
// Created by wnabo on 08.03.2019.
//

#ifndef IKALGORITHMS_FABRIKSOLVE_H
#define IKALGORITHMS_FABRIKSOLVE_H


#include "AcquisitionChain.h"
#include <Eigen/Dense>
#include <vector>

class FabrikSolve {
public:
    FabrikSolve(std::vector<Eigen::Vector3d> joints, Eigen::Vector3d target, Eigen::Vector3d origin,
                float totalLength, std::vector<float> distances, float tolerance, bool isConstrained);

    FabrikSolve(AcquisitionChain chain, float tolerance, bool isConstrained);

    virtual ~FabrikSolve();

    void solve();

    const std::vector<Eigen::Vector3d> &getJoints() const;

    void setJointAt(Eigen::Vector3d newJoint, int index);

    void setTarget(const Eigen::Vector3d &target);

    void setOrigin(const Eigen::Vector3d &origin);

    void chain_backwards();

    void chain_forwards();
    void setAllConeConstraints(const float up, const float down, const float left, const float right);

    Eigen::Vector3d
    chain_constrain(Eigen::Vector3d calc, Eigen::Vector3d line, Eigen::Matrix4d direction);

    void calcConeDirection(Eigen::Vector3d joint, Eigen::Matrix4d cf);

private:
    std::vector<Eigen::Vector3d> joints;
    std::vector<std::pair<Eigen::Vector3d, std::vector<double> > > jointAngleConstraintPairs;
    Eigen::Vector3d target;
    Eigen::Vector3d origin;
    Eigen::Vector3d rightvec;
    Eigen::Vector3d upvec;
    float totalLength;
    std::vector<float> distances;
    float tolerance;
    bool isConstrained;
    float right;
    float left;
    float up;
    float down;
};


#endif //IKALGORITHMS_FABRIKSOLVE_H
