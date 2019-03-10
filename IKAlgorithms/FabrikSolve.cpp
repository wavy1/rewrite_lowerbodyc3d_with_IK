//
// Created by wnabo on 08.03.2019.
//

#include "FabrikSolve.h"
#include <iostream>

void FabrikSolve::chain_backwards(){

    joints.at(joints.size()-1) = target;
    float r;
    float l;
    Eigen::Vector3f position;
    for(size_t index = joints.size()-2; index > 0;--index){

        r = (*joints.at(index+1) - *joints.at(index)).norm();
        l = *distances.at(index) / r;

        position = (1 - l) * *joints.at(index + 1) + l * *joints.at(index);
        joints.at(index) = std::make_shared<Eigen::Vector3f>(position);
    }
}

void FabrikSolve::chain_forwards() {
    joints.at(0) = origin;
    Eigen::Vector3f position;
    float r;
    float l;
    for(size_t index = 0; index < joints.size()-2; ++index){
        r = (*joints.at(index+1) - *joints.at(index)).norm();
        l = *distances.at(index) / r;

        position = (1 - l) * *joints.at(index) + l * *joints.at(index+1);
        joints.at(index+1) = std::make_shared<Eigen::Vector3f>(position);
    }
}

void FabrikSolve::solve() {
    float distance = (*joints.at(joints.size() - 1) - *target).norm();
    float r;
    float l;
    float dif;
    int bcount;
    if (distance > totalLength) {
        for (size_t index = 0; index < joints.size(); ++index) {
            r = (*target - *joints.at(index)).norm();
            l = *distances.at(index) / r;
            if((index + 1) < joints.size())
                joints.at(index + 1) = std::make_shared<Eigen::Vector3f>(((1 - l) * *joints.at(index)) + (l * *target));
        }
    } else {
        bcount = 0;
        dif = (*joints.at(joints.size()-1) - *target).norm();
        while (dif > tolerance) {
            chain_backwards();
            chain_forwards();
            dif = (*joints.at(joints.size() - 1) - *target).norm();

            bcount = bcount + 1;
            if (bcount > 10) {
                break;
            }
        }
    }
}

FabrikSolve::FabrikSolve(std::vector<std::shared_ptr<Eigen::Vector3f> >  joints, std::shared_ptr<Eigen::Vector3f> target,
                         std::shared_ptr<Eigen::Vector3f> origin, float totalLength, std::vector<std::shared_ptr<float> > distances,
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

