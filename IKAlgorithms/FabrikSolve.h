//
// Created by wnabo on 08.03.2019.
//

#ifndef IKALGORITHMS_FABRIKSOLVE_H
#define IKALGORITHMS_FABRIKSOLVE_H


#include <Eigen/Dense>
#include <vector>

class FabrikSolve {
public:
    FabrikSolve(std::vector<Eigen::Vector3f> joints, Eigen::Vector3f target, Eigen::Vector3f origin,
                float totalLength, std::vector<float> distances, float tolerance);

    virtual ~FabrikSolve();

    void solve();

private:
    void chain_backwards();
    void chain_forwards();
    std::vector<Eigen::Vector3f> joints;
    Eigen::Vector3f target;
    Eigen::Vector3f origin;
    float totalLength;
    std::vector<float> distances;
    float tolerance;
};


#endif //IKALGORITHMS_FABRIKSOLVE_H
