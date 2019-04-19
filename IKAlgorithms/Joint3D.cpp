//
// Created by wnabo on 06.04.2019.
//

#include "Joint3D.h"

Joint3D::Joint3D(Eigen::Vector3d position, double angleLeft, double angleRight, double angleUp, double angleDown)
        : position(position), constraintAngleLeft(angleLeft), constraintAngleRight(angleRight),
          constraintAngleUp(angleUp), constraintAngleDown(angleDown){};

Joint3D::~Joint3D() {}