//
// Created by wnabo on 08.03.2019.
//

#ifndef IKALGORITHMS_FABRIKSOLVE_H
#define IKALGORITHMS_FABRIKSOLVE_H


#include <Eigen/Dense>
#include <vector>
#include <memory>

class FabrikSolve {
public:
    FabrikSolve(std::vector<std::shared_ptr<Eigen::Vector3f> >  joints, std::shared_ptr<Eigen::Vector3f> target,
                std::shared_ptr<Eigen::Vector3f> origin, float totalLength, std::vector<std::shared_ptr<float> > distances,
                float tolerance);

    virtual ~FabrikSolve();

    void solve();

private:
    void chain_backwards();
    void chain_forwards();
    std::vector<std::shared_ptr<Eigen::Vector3f> > joints;
    std::shared_ptr<Eigen::Vector3f> target;
    std::shared_ptr<Eigen::Vector3f> origin;
    float totalLength;
    std::vector<std::shared_ptr<float> > distances;
    float tolerance;
};


#endif //IKALGORITHMS_FABRIKSOLVE_H
