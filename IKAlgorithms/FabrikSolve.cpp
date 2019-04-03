//
// Created by wnabo on 08.03.2019.
//

#include "FabrikSolve.h"
#include <iostream>
#include <math.h>

void FabrikSolve::chain_backwards() {
    std::cout << "Chain backward" << std::endl;
    std::cout << "Last Joint: [" << joints.at(joints.size() - 1).coeff(0) << ", "
              << joints.at(joints.size() - 1).coeff(1) << ", " << joints.at(joints.size() - 1).coeff(2) << "]"
              << std::endl;
    joints.at(joints.size() - 1) = target;
    std::cout << "Last Joint: [" << joints.at(joints.size() - 1).coeff(0) << ", "
              << joints.at(joints.size() - 1).coeff(1) << ", " << joints.at(joints.size() - 1).coeff(2) << "]"
              << std::endl;
    std::cout << "Target: [" << target.coeff(0) << ", " << target.coeff(1) << ", " << target.coeff(2) << "]"
              << std::endl;
    float r;
    float l;
    Eigen::Vector3d position;
    for (size_t index = joints.size() - 2; index > 0; --index) {

        std::cout << "Joint index+1: [" << joints.at(index + 1).coeff(0) << ", " << joints.at(index + 1).coeff(1)
                  << ", " << joints.at(index + 1).coeff(2) << "]" << std::endl;
        std::cout << "Joint index: [" << joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1) << ", "
                  << joints.at(index).coeff(2) << "]" << std::endl;
        r = (joints.at(index + 1) - joints.at(index)).norm();
        std::cout << "r: " << r << std::endl;
        l = distances.at(index) / r;
        std::cout << "l: " << l << std::endl;

        position = (1 - l) * joints.at(index + 1) + l * joints.at(index);
        std::cout << "Position: [" << position.coeff(0) << ", " << position.coeff(1) << ", " << position.coeff(2) << "]"
                  << std::endl;
        std::cout << "IndexAtAlgorithm: " << index << std::endl;
        joints.at(index) = position;
    }
}

float degToRad(float degrees) {
    return degrees * M_PI / 180;
}

void FabrikSolve::chain_forwards() {
    std::cout << "Chain forwards" << std::endl;
    std::cout << "Origin: [" << origin.coeff(0) << ", " << origin.coeff(1) << ", " << origin.coeff(2) << "]"
              << std::endl;
    std::cout << "First Joint: [" << joints.at(0).coeff(0) << ", " << joints.at(0).coeff(1) << ", "
              << joints.at(0).coeff(2) << "]" << std::endl;
    joints.at(0) = origin;
    Eigen::Vector3d position;
    Eigen::Vector3d coneVec;
    if (joints.size() >= 2) {
        coneVec = (joints.at(1) - joints.at(0)).normalized();
    }
    float r;
    float l;
    Eigen::Vector3d t;
    for (size_t index = 0; index < joints.size() - 2; ++index) {
        std::cout << "Joint index+1: [" << joints.at(index + 1).coeff(0) << ", " << joints.at(index + 1).coeff(1)
                  << ", " << joints.at(index + 1).coeff(2) << "]" << std::endl;
        std::cout << "Joint index: [" << joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1) << ", "
                  << joints.at(index).coeff(2) << "]" << std::endl;
        r = (joints.at(index + 1) - joints.at(index)).norm();
        std::cout << "r: " << r << std::endl;
        l = distances.at(index) / r;
        std::cout << "l: " << l << std::endl;

        Eigen::Vector3d lookAt = joints.at(index) + coneVec;
        Eigen::Vector3d coneOrigin = joints.at(index);
        Eigen::Vector3d objectUpVector(0, 0, 1);

        Eigen::Vector3d zaxis = Eigen::Vector3d(lookAt - coneOrigin).normalized();
        Eigen::Vector3d xaxis = Eigen::Vector3d(objectUpVector.cross(zaxis)).normalized();
        Eigen::Vector3d yaxis = Eigen::Vector3d(zaxis.cross(xaxis));

        Eigen::Matrix4d cf;

        cf << xaxis.coeff(0), xaxis.coeff(1), xaxis.coeff(2), 0,
                yaxis.coeff(0), yaxis.coeff(1), yaxis.coeff(2), 0,
                zaxis.coeff(0), zaxis.coeff(1), zaxis.coeff(2), 0,
                0, 0, 0, 1;

        position = (1 - l) * joints.at(index) + l * joints.at(index + 1);
        if (isConstrained) {
            t = chain_constrain(position - joints.at(index), coneVec, cf);
            joints.at(index + 1) = joints.at(index) + t;
        } else {
            joints.at(index + 1) = position;
        }
        std::cout << "Position: [" << position.coeff(0) << ", " << position.coeff(1) << ", " << position.coeff(2) << "]"
                  << std::endl;
        std::cout << "Index: " << index << std::endl;
        if (joints.size() > index) {
            coneVec = (joints.at(index + 1) - joints.at(index)).normalized();
        }
    }
}

void FabrikSolve::solve() {
    std::cout << "Before Solve" << std::endl;
    for (size_t index = 0; index < joints.size(); ++index) {
        std::cout << "Joint index: (" << index << ")[" << joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1)
                  << ", " << joints.at(index).coeff(2) << "]" << std::endl;
    }
    std::cout << "Target: [" << target.coeff(0) << ", " << target.coeff(1) << ", " << target.coeff(2) << "]"
              << std::endl;
    float distance = (joints.at(joints.size() - 1) - target).norm();
    float r;
    float l;
    float dif;
    int bcount;
    std::cout << "Distance: " << distance << " vs. total Lenght: " << totalLength << std::endl;
    std::cout << "Distances Size:" << distances.size() << std::endl;
    std::cout << "Joints Size:" << joints.size() << std::endl;
    std::cout << "Distances Number: " << distances.size() << std::endl;
    if (distance > totalLength) {
        for (size_t index = 0; index < joints.size(); ++index) {
            r = (target - joints.at(index)).norm();
            l = distances.at(index) / r;
            if ((index + 1) < joints.size())
                joints.at(index + 1) = ((1 - l) * joints.at(index)) + (l * target);
        }
    } else {

        bcount = 0;
        dif = (joints.at(joints.size() - 1) - target).norm();
        std::cout << "Dif: " << dif << " vs. tolerance: " << tolerance << std::endl;
        while (dif > tolerance) {
            chain_backwards();
            chain_forwards();
            dif = (joints.at(joints.size() - 1) - target).norm();

            bcount = bcount + 1;
            if (bcount > 10) {
                break;
            }
        }
    }
    std::cout << "After Solve" << std::endl;
    for (size_t index = 0; index < joints.size(); ++index) {
        std::cout << "Joint index: (" << index << ")[" << joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1)
                  << ", " << joints.at(index).coeff(2) << "]" << std::endl;
    }
    std::cout << std::endl;
}

FabrikSolve::~FabrikSolve() {
    std::cout << "Destructor for Solver has been executed\n" << std::endl;
}

const std::vector<Eigen::Vector3d> &FabrikSolve::getJoints() const {
    return joints;
}

void FabrikSolve::setTarget(const Eigen::Vector3d &target) {
    FabrikSolve::target = target;
}

void FabrikSolve::setOrigin(const Eigen::Vector3d &origin) {
    FabrikSolve::joints.at(0) = origin;
    FabrikSolve::origin = origin;
}

void FabrikSolve::setJointAt(Eigen::Vector3d newJoint, int index) {
    FabrikSolve::joints.at(index) = newJoint;
}

Eigen::Vector3d
FabrikSolve::chain_constrain(Eigen::Vector3d calc, Eigen::Vector3d line, Eigen::Matrix4d direction) {
    std::cout << "Constrain" << std::endl;
    float scalar = calc.dot(line) / line.norm();
    Eigen::Vector3d proj = scalar * line.normalized();

    calcConeDirection(calc, direction);
    Eigen::Vector3d adjust = calc - proj;
    if (scalar < 0) {
        proj = -proj;
    }

    float xaspect = adjust.dot(rightvec);
    float yaspect = adjust.dot(upvec);

    float left = -(proj.norm() * std::tan(this->left));
    float right = proj.norm() * std::tan(this->right);
    float up = proj.norm() * std::tan(this->up);
    float down = -(proj.norm() * std::tan(this->down));

    float xbound = xaspect >= 0 ? right : left;
    float ybound = yaspect >= 0 ? up : down;

    Eigen::Vector3d f = calc;

    float ellipse = std::sqrt(xaspect) / std::sqrt(xbound) + std::sqrt(yaspect) / std::sqrt(ybound);
    bool inbounds = ellipse <= 1 && scalar >= 0;

    if (!inbounds) {
        std::cout << "Not in bounds, adjust" << std::endl;
        float a = std::atan2(yaspect, xaspect);

        float x = xbound * std::cos(a);
        float y = ybound * std::sin(a);

        f = (proj + rightvec * x + upvec * y).normalized() * calc.norm();
    }

    return f;
}

void FabrikSolve::calcConeDirection(Eigen::Vector3d calc, Eigen::Matrix4d cf) {

    Eigen::Vector3d jointToTarget = calc;

    Eigen::Vector3d upVector = Eigen::Vector3d(cf.coeff(0, 2), cf.coeff(1, 2), cf.coeff(2, 2));
    Eigen::Vector3d downVector = Eigen::Vector3d(-cf.coeff(0, 2), -cf.coeff(1, 2), -cf.coeff(2, 2));
    Eigen::Vector3d rightVector = Eigen::Vector3d(cf.coeff(0, 0), cf.coeff(1, 0), cf.coeff(2, 0));
    Eigen::Vector3d leftVector = Eigen::Vector3d(-cf.coeff(0, 0), -cf.coeff(1, 0), -cf.coeff(2, 0));


    std::cout << "Direction Vector to target:("
              << jointToTarget.coeff(0) << ", " << jointToTarget.coeff(1) << ", " << jointToTarget.coeff(2) << ")"
              << std::endl;
    if ((upVector - jointToTarget).norm() > (downVector - jointToTarget).norm()) {
        upvec = upVector;
    } else if ((upVector - jointToTarget).norm() < (downVector - jointToTarget).norm()) {
        upvec = downVector;
        std::cout << "Up" << std::endl;
    } else {
        std::cout << "Fuck Deadlock" << std::endl;
    }
    std::cout << "x: " << upvec.coeff(0) << " y: " << upvec.coeff(1) << " z: " << upvec.coeff(2) << std::endl;

    if ((leftVector - jointToTarget).norm() > (rightVector - jointToTarget).norm()) {
        std::cout << "Left" << std::endl;
        rightvec = leftVector;
    } else if ((leftVector - jointToTarget).norm() < (rightVector - jointToTarget).norm()) {
        std::cout << "Right" << std::endl;
        rightvec = rightVector;
    } else {
        std::cout << "Fuck Deadlock" << std::endl;
    }
    std::cout << "x: " << rightvec.coeff(0) << " y: " << rightvec.coeff(1) << " z: " << rightvec.coeff(2) << std::endl;
}

void FabrikSolve::setAllConeConstraints(const float right, const float left, const float up, const float down) {
    this->right = degToRad(right);
    this->left = degToRad(left);
    this->up = degToRad(up);
    this->down = degToRad(down);
}

FabrikSolve::FabrikSolve(std::vector<Eigen::Vector3d> joints, Eigen::Vector3d target, Eigen::Vector3d origin,
                         float totalLength, std::vector<float> distances, float tolerance, bool isConstrained) {
    this->joints = joints;
    this->target = target;
    this->origin = origin;
    this->distances = distances;
    this->tolerance = tolerance;
    this->totalLength = totalLength;
    this->isConstrained = isConstrained;

    std::cout << "Constructor is executed\n" << std::endl;
}

FabrikSolve::FabrikSolve(AcquisitionChain chain, float tolerance, bool isConstrained) {
    this->joints = chain.getPositionsPerFrame(0);
    this->target = *chain.getPositionsPerFrame(0).rbegin();
    this->origin = *chain.getPositionsPerFrame(0).begin();
    this->distances = chain.getDistances();
    this->tolerance = tolerance;
    this->totalLength = chain.getSetSumOfAllLenghts();
    this->isConstrained = isConstrained;

}

