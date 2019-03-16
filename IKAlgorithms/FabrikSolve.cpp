//
// Created by wnabo on 08.03.2019.
//

#include "FabrikSolve.h"
#include <vector>
#include <iostream>

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
    Eigen::Vector3f position;
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
        std::cout << "Index: " << index << std::endl;
        joints.at(index) = position;
    }
}

void FabrikSolve::chain_forwards() {
    std::cout << "Chain forwards" << std::endl;
    std::cout << "Origin: [" << origin.coeff(0) << ", " << origin.coeff(1) << ", " << origin.coeff(2) << "]"
              << std::endl;
    std::cout << "First Joint: [" << joints.at(0).coeff(0) << ", " << joints.at(0).coeff(1) << ", "
              << joints.at(0).coeff(2) << "]" << std::endl;
    joints.at(0) = origin;
    Eigen::Vector3f position;
    float r;
    float l;
    for (size_t index = 0; index < joints.size() - 2; ++index) {
        std::cout << "Joint index+1: [" << joints.at(index + 1).coeff(0) << ", " << joints.at(index + 1).coeff(1)
                  << ", " << joints.at(index + 1).coeff(2) << "]" << std::endl;
        std::cout << "Joint index: [" << joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1) << ", "
                  << joints.at(index).coeff(2) << "]" << std::endl;
        r = (joints.at(index + 1) - joints.at(index)).norm();
        std::cout << "r: " << r << std::endl;
        l = distances.at(index) / r;
        std::cout << "l: " << l << std::endl;

        position = (1 - l) * joints.at(index) + l * joints.at(index + 1);
        std::cout << "Position: [" << position.coeff(0) << ", " << position.coeff(1) << ", " << position.coeff(2) << "]"
                  << std::endl;
        std::cout << "Index: " << index << std::endl;
        joints.at(index + 1) = position;
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

FabrikSolve::FabrikSolve(std::vector<Eigen::Vector3f> joints, Eigen::Vector3f target,
                         Eigen::Vector3f origin, float totalLength, std::vector<float> distances,
                         float tolerance) {
    this->joints = joints;
    this->target = target;
    this->origin = origin;
    this->distances = distances;
    this->tolerance = tolerance;
    this->totalLength = totalLength;
    std::cout << "Constructor is executed\n" << std::endl;
}

FabrikSolve::~FabrikSolve() {
    std::cout << "Destructor is executed\n" << std::endl;
}

const std::vector<Eigen::Vector3f> &FabrikSolve::getJoints() const {
    return joints;
}

void FabrikSolve::setTarget(const Eigen::Vector3f &target) {
    FabrikSolve::target = target;
}

void FabrikSolve::setOrigin(const Eigen::Vector3f &origin) {
    FabrikSolve::origin = origin;
}

void FabrikSolve::setJointAt(Eigen::Vector3f newJoint, int index) {
    FabrikSolve::joints.at(index) = newJoint;
}

void FabrikSolve::calcConeDirection(Eigen::Vector3f joint, Eigen::Vector3f target) {
    Eigen::Vector3f jointToTarget = target - joint;
    Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f down = Eigen::Vector3f(0, -1, 0);
    Eigen::Vector3f left = Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f right = Eigen::Vector3f(-1, 0, 0);

    if ((joint + up).norm() - jointToTarget.norm() > (joint + down).norm() - jointToTarget.norm()) {
        upvec = joint + down;
        std::cout << "Down" << std::endl;
    } else if ((joint + up).norm() - jointToTarget.norm() < (joint + down).norm() - jointToTarget.norm()) {
        upvec = joint + up;
        std::cout << "Up" << std::endl;
    } else {
        std::cout << "Fuck Deadlock" << std::endl;
    }

    if ((joint + left).norm() - jointToTarget.norm() > (joint + right).norm() - jointToTarget.norm()) {
        std::cout << "Left" << std::endl;
        rightvec = joint + left;
    } else if ((joint + left).norm() - jointToTarget.norm() < (joint + right).norm() - jointToTarget.norm()) {
        std::cout << "Left" << std::endl;
        rightvec = joint + right;
    } else {
        std::cout << "Fuck Deadlock" << std::endl;
    }
}

Eigen::Vector3f chain_constrain(Eigen::Vector3f calc, Eigen::Vector3f line, Eigen::Vector3f joint, std::string direction){
    float scalar = calc.dot(line) / line.norm();
    Eigen::Vector3f proj = scalar * line.normalized();

    Eigen::Vector3f adjust = calc - proj;

    float xaspect = adjust.dot(Eigen::Vector3f::UnitX());
    float yaspect = adjust.dot(Eigen::Vector3f::UnitY());

    float left = -(proj.norm() * std::tan());
    float right = proj.norm() * std::tan();
    float up = proj.norm() * std::tan();
    float down = -(proj.norm() * std::tan());

    float xbound = xaspect >=0 and right or left;
    float ybound = yaspect >=0 and up or down;

    Eigen::Vector3f f = calc;

    float ellipse = std::sqrt(xaspect)/std::sqrt(xbound) + std::sqrt(yaspect)/std::sqrt(ybound);
    float inbounds = ellipse <= 1;

    if(! inbounds ){
        float a = std::atan2(yaspect, xaspect);

        float x = xbound * std::cos(a);
        float y = ybound * std::sin(a);

        f = (proj + rightvec * x + upvec * y).normalized * calc.norm();
    }

    return f;

}

