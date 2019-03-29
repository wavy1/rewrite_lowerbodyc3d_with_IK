//
// Created by wnabo on 28.03.2019.
//

#ifndef IKALGORITHMS_BTKPOINTREADADAPTER_H
#define IKALGORITHMS_BTKPOINTREADADAPTER_H


#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <Eigen/Dense>
#include <map>

class AcqPointReadAdapter {
public:
    Eigen::Vector3d pointAt(btk::Point::Pointer itP, int frameNumber);
    Eigen::Vector3d pointAt(std::string pointStr, int frameNumber);
    void addToMap(std::string str, btk::Point::Pointer acqPoint, bool includeDebuggable = false, std::string debugStr = "");
    std::vector<Eigen::Vector3d> getPositionsPerFrame(int frameNumber);
    std::vector<btk::Point::Pointer> getPoints();
    std::vector<std::pair<std::string, Eigen::Vector3d> > getDebuggablePairsVectorOfFrame(int frameNumber);

private:
    std::map<std::string, btk::Point::Pointer> stringToPointMap;
    std::map<std::string, std::pair<std::string, btk::Point::Pointer> > stringToPointMapDebuggable;
};


#endif //IKALGORITHMS_BTKPOINTREADADAPTER_H
