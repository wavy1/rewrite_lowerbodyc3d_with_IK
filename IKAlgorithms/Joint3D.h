//
// Created by wnabo on 06.04.2019.
//

#ifndef IKALGORITHMS_JOINT_H
#define IKALGORITHMS_JOINT_H

#include <Eigen/Dense>

struct Joint3D {
    Joint3D(Eigen::Vector3d position, double angleLeft, double angleRight, double angleUp, double angleDown);
    virtual ~Joint3D();
    Eigen::Vector3d position;
    double constraintAngleLeft;
    double constraintAngleRight;
    double constraintAngleUp;
    double constraintAngleDown;
};


#endif //IKALGORITHMS_JOINT_H
