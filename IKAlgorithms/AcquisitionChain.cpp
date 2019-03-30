//
// Created by wnabo on 28.03.2019.
//

#include <vector>
#include "AcquisitionChain.h"

Eigen::Vector3d AcquisitionChain::pointAt(btk::Point::Pointer itP, int frameNumber) {
    Eigen::Vector3d point(itP->GetValues().coeff(frameNumber, 0),
                          itP->GetValues().coeff(frameNumber, 1),
                          itP->GetValues().coeff(frameNumber, 2));
    return point;
}

Eigen::Vector3d AcquisitionChain::pointAt(std::string pointStr, int frameNumber) {

    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix = this->stringToPointMap[pointStr]->GetValues();

    Eigen::Vector3d point(matrix.coeff(frameNumber, 0),
                          matrix.coeff(frameNumber, 1),
                          matrix.coeff(frameNumber, 2));
    return point;
}

void AcquisitionChain::addToMap(std::string str, btk::Point::Pointer acqPoint, bool includeDebuggable,
                                   std::string debugStr) {
    this->stringToPointMap[str] = acqPoint;

    if (includeDebuggable) {
        this->stringToPointMapDebuggable[str] = std::make_pair(debugStr, acqPoint);
    }
}

std::vector<btk::Point::Pointer> AcquisitionChain::getPoints() {
    std::vector<btk::Point::Pointer> my_vals;

    for(std::map<std::string, btk::Point::Pointer>::iterator peter = stringToPointMap.begin(); peter != stringToPointMap.end(); ++peter){
        my_vals.push_back(peter->second);
    }
    /* std::vector<btk::Point::Pointer> my_vals;
    std::transform(this->stringToPointMap.begin(), this->stringToPointMap.end(), std::back_inserter(my_vals),
                   [this](const std::pair<std::string, btk::Point::Pointer> &val) -> btk::Point::Pointer {
                       return val.second;
                   });
    return my_vals;
     */
}

std::vector<Eigen::Vector3d> AcquisitionChain::getPositionsPerFrame(int frameNumber) {
    std::vector<Eigen::Vector3d> my_vals;

    for(std::map<std::string, btk::Point::Pointer>::iterator peter = stringToPointMap.begin(); peter != stringToPointMap.end(); ++peter){
        my_vals.push_back(pointAt(peter->second, frameNumber));
    }

    /*
    std::transform(this->stringToPointMap.begin(), this->stringToPointMap.end(), std::back_inserter(my_vals),
                   [this, frameNumber](const std::pair<std::string, btk::Point::Pointer> &val) -> Eigen::Vector3d {
                       return pointAt(val.second, frameNumber);
                   });
     */
    return my_vals;
}

std::vector<std::pair<std::string, Eigen::Vector3d> >
AcquisitionChain::getDebuggablePairsVectorOfFrame(int frameNumber) {

    std::vector<std::pair<std::string, Eigen::Vector3d> > my_map;

    for(std::map<std::string, std::pair<std::string, btk::Point::Pointer> >::iterator ptr = stringToPointMapDebuggable.begin(); ptr != stringToPointMapDebuggable.end(); ++ptr){
        my_map.push_back(std::make_pair(ptr->second.first, pointAt(ptr->second.second, frameNumber)));
    }

    /*
    std::transform(this->stringToPointMapDebuggable.begin(), this->stringToPointMapDebuggable.end(),
                   std::back_inserter(my_map), [this, frameNumber](
                    const std::pair<std::string, std::pair<std::string, btk::Point::Pointer> > &val) -> std::pair<std::string, Eigen::Vector3d> {
                return std::make_pair(val.second.first, pointAt(val.second.second, frameNumber));
            });
     */
    return my_map;
}

void AcquisitionChain::calculateDistancesDebugless(std::vector<Eigen::Vector3d> joints) {

    this->sumOfAllLenghts = 0;

    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            this->distances.insert(std::make_pair("P" + index, (joints.at(index + 1) - joints.at(index)).norm()));
            this->sumOfAllLenghts += (joints.at(index + 1) - joints.at(index)).norm();
        }
    }
}

float AcquisitionChain::getSetSumOfAllLenghts() {
    return this->sumOfAllLenghts;
}

std::vector<float> AcquisitionChain::getDistances() {
    std::vector<float> distances;

    for(std::map<std::string, float>::iterator distanceMapIt = this->distances.begin(); distanceMapIt != this->distances.end(); ++distanceMapIt){
        distances.push_back(distanceMapIt->second);
    }
    return distances;
}
