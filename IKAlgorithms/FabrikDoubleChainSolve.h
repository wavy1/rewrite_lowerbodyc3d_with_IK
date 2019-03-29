//
// Created by wnabo on 15.03.2019.
//

#ifndef IKALGORITHMS_FABRIKDOUBLECHAINSOLVE_H
#define IKALGORITHMS_FABRIKDOUBLECHAINSOLVE_H

#include <Eigen/Dense>
#include <vector>

class FabrikDoubleChainSolve {
public:
    FabrikDoubleChainSolve(std::vector<Eigen::Vector3f> joints, Eigen::Vector3f target, Eigen::Vector3f origin,
    float totalLength, std::vector<float> distances, float tolerance);
    virtual ~FabrikDoubleChainSolve();
    void solve();
    const std::vector<Eigen::Vector3f> &getJoints() const;
    void setTarget(const Eigen::Vector3f &target);
    void chain_backwards_1();
    void chain_forwards_1();
    void chain_backwards_2();
    void chain_forwards_2();

private:
    std::vector<Eigen::Vector3f> joints1;
    Eigen::Vector3f target1;
    Eigen::Vector3f origin1;
    float totalLength1;
    std::vector<float> distances1;

    std::vector<Eigen::Vector3f> joints2;
    Eigen::Vector3f target2;
    Eigen::Vector3f origin2;
    float totalLength2;
    std::vector<float> distances2;
    float tolerance;
};


#endif //IKALGORITHMS_FABRIKDOUBLECHAINSOLVE_H
