//
// Created by wnabo on 28.03.2019.
//

#include <vector>
#include <iostream>
#include "AcquisitionChain.h"

Eigen::Vector3d AcquisitionChain::pointAt(btk::Point::Pointer itP, int frameNumber) {
    Eigen::Vector3d point(itP->GetValues().coeff(frameNumber, 0),
                          itP->GetValues().coeff(frameNumber, 1),
                          itP->GetValues().coeff(frameNumber, 2));
    return point;
}

Eigen::Vector3d AcquisitionChain::pointAt(std::string pointStr, int frameNumber) {

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix;

    for (std::vector<std::pair<std::string, btk::Point::Pointer> >::iterator strToPIt = this->stringToPointMap.begin();
         strToPIt != this->stringToPointMap.end(); ++strToPIt) {
        if (strToPIt->first == pointStr) {
            matrix = strToPIt->second->GetValues();
        }
    }

    Eigen::Vector3d point(matrix.coeff(frameNumber, 0),
                          matrix.coeff(frameNumber, 1),
                          matrix.coeff(frameNumber, 2));
    return point;
}

void AcquisitionChain::addToMap(std::string str, btk::Point::Pointer acqPoint, bool includeDebuggable,
                                std::string debugStr) {
    this->stringToPointMap.push_back(std::make_pair(str, acqPoint));

    if (includeDebuggable) {
        this->stringToPointMapDebuggable.push_back(std::make_pair(str, std::make_pair(debugStr, acqPoint)));
    }
}

std::vector<btk::Point::Pointer> AcquisitionChain::getPoints() {
    std::vector<btk::Point::Pointer> my_vals;

    for (std::vector<std::pair<std::string, btk::Point::Pointer> >::iterator peter = stringToPointMap.begin();
         peter != stringToPointMap.end(); ++peter) {
        my_vals.push_back(peter->second);
    }
    return my_vals;
}

std::vector<Eigen::Vector3d> AcquisitionChain::getPositionsPerFrame(int frameNumber) {
    std::vector<Eigen::Vector3d> my_vals;

    for (std::vector<std::pair<std::string, btk::Point::Pointer> >::iterator peter = stringToPointMap.begin();
         peter != stringToPointMap.end(); ++peter) {
        std::cout << peter->first << std::endl;
        my_vals.push_back(pointAt(peter->second, frameNumber));
    }

    return my_vals;
}

std::vector<std::pair<std::string, Eigen::Vector3d> >
AcquisitionChain::getDebuggablePairsVectorOfFrame(int frameNumber) {

    std::vector<std::pair<std::string, Eigen::Vector3d> > my_map;

    for (std::vector<std::pair<std::string, std::pair<std::string, btk::Point::Pointer> > >::iterator ptr = stringToPointMapDebuggable.begin();
         ptr != stringToPointMapDebuggable.end(); ++ptr) {
        my_map.push_back(std::make_pair(ptr->second.first, pointAt(ptr->second.second, frameNumber)));
    }

    return my_map;
}

void AcquisitionChain::calculateDistancesDebugless(std::vector<Eigen::Vector3d> joints) {

    this->sumOfAllLenghts = 0;
    std::cout << "[AcqChain] Size of Joints" << joints.size() << std::endl;
    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            std::cout << "[AcqChain]" << " distance for " << "P" << index << ":[" <<
                      joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1) << ", "
                      << joints.at(index).coeff(2) << std::endl;
            std::cout << "[AcqChain]" << " distance for " << "P" << index + 1 << ":[" <<
                      joints.at(index + 1).coeff(0) << ", " << joints.at(index + 1).coeff(1) << ", "
                      << joints.at(index + 1).coeff(2) << std::endl;

            this->distances.push_back(std::make_pair("P" + index, (joints.at(index + 1) - joints.at(index)).norm()));
            this->sumOfAllLenghts += (joints.at(index + 1) - joints.at(index)).norm();
        }
    }
    std::cout << "[AcqChain] distance" << this->distances.size() << std::endl;
}

std::vector<double> calculateDistances(std::vector<Eigen::Vector3d> joints) {
    std::vector<double> distances;

    std::cout << "[AcqChain] Size of Joints" << joints.size() << std::endl;
    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            std::cout << "[AcqChain]" << " distance for " << "P" << index << ":[" <<
                      joints.at(index).coeff(0) << ", " << joints.at(index).coeff(1) << ", "
                      << joints.at(index).coeff(2) << std::endl;
            std::cout << "[AcqChain]" << " distance for " << "P" << index + 1 << ":[" <<
                      joints.at(index + 1).coeff(0) << ", " << joints.at(index + 1).coeff(1) << ", "
                      << joints.at(index + 1).coeff(2) << std::endl;

            distances.push_back((joints.at(index + 1) - joints.at(index)).norm());
        }
    }
    return distances;
}

void AcquisitionChain::writeDistancesIntoCSV(std::vector<Eigen::Vector3d> joints, std::string filename, int frameNumber, bool isEndLine){
    std::ofstream distancesCSV;

    distancesCSV.open(filename.c_str(), std::ios::app | std::ios::out);
    std::vector<double> distances = calculateDistances(joints);

    for(size_t index = 0; index < distances.size(); ++index) {
        if(!isEndLine && index == 0){
            distancesCSV << frameNumber << ";";
        }
        if (isEndLine && index + 1  == distances.size()) {
            distancesCSV << distances.at(index) << "\n";
            break;
        }
        distancesCSV << distances.at(index) << ";";
    }

    distancesCSV.close();
}

float AcquisitionChain::getSetSumOfAllLenghts() {
    return this->sumOfAllLenghts;
}

std::vector<float> AcquisitionChain::getDistances() {
    std::vector<float> distances;

    for (std::vector<std::pair<std::string, float> >::iterator distanceMapIt = this->distances.begin();
         distanceMapIt != this->distances.end(); ++distanceMapIt) {
        distances.push_back(distanceMapIt->second);
    }
    return distances;
}

void AcquisitionChain::addPointWithConstraints(std::pair<btk::Point::Pointer, std::vector<double> > pointWithAngles) {
    this->pointWithAngleConstraints.push_back(pointWithAngles);
}

std::vector<std::pair<Eigen::Vector3d, std::vector<double> > > AcquisitionChain::getJointsWithAngleConstraints() {
    std::vector<std::pair<Eigen::Vector3d, std::vector<double> > > positionsWithAngleConstraints;
    std::vector<std::pair<btk::Point::Pointer, std::vector<double> > > points = this->pointWithAngleConstraints;
    for (std::vector<std::pair<btk::Point::Pointer, std::vector<double> > >::iterator pointsIterator = points.begin();
         pointsIterator != points.end(); ++pointsIterator) {
        std::cout << pointsIterator->first->GetLabel() << ": " << " Up: " << pointsIterator->second.at(0) << " Down: " <<
        pointsIterator->second.at(1) << " Left " << pointsIterator->second.at(2) << " Right "
                                     << pointsIterator->second.at(3) << std::endl;
        positionsWithAngleConstraints.push_back(
                std::make_pair(pointAt(pointsIterator->first, 0), pointsIterator->second));
    }
    return positionsWithAngleConstraints;
}

AcquisitionChain::~AcquisitionChain() {
    std::cout << "Destructor for Chain has been executed\n" << std::endl;
}
