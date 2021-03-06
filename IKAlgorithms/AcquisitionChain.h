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

class AcquisitionChain {
public:
    virtual ~AcquisitionChain();

    Eigen::Vector3d pointAt(btk::Point::Pointer itP, int frameNumber);

    Eigen::Vector3d pointAt(std::string pointStr, int frameNumber);

    void
    addToMap(std::string str, btk::Point::Pointer acqPoint, bool includeDebuggable = false, std::string debugStr = "");

    void
    addPointWithConstraints(std::pair<btk::Point::Pointer, std::vector<double> > pointWithAngles);

    std::vector<Eigen::Vector3d> getPositionsPerFrame(int frameNumber);

    std::vector<btk::Point::Pointer> getPoints();

    std::vector<std::pair<std::string, Eigen::Vector3d> > getDebuggablePairsVectorOfFrame(int frameNumber);

    void calculateDistancesDebugless(std::vector<Eigen::Vector3d> joints);

    void writeDistancesIntoCSV(std::vector<Eigen::Vector3d> joints, std::string filename, int frameNumber, bool isEndLine);

    float getSetSumOfAllLenghts();

    std::vector<float> getDistances();

    std::vector<std::pair< Eigen::Vector3d, std::vector<double> > > getJointsWithAngleConstraints();


private:
    std::vector<std::pair<std::string, float> > distances;
    float sumOfAllLenghts;
    std::vector<std::pair<std::string, btk::Point::Pointer> > stringToPointMap;
    std::vector<std::pair<std::string, std::pair<std::string, btk::Point::Pointer> > > stringToPointMapDebuggable;
    std::vector<std::pair< btk::Point::Pointer, std::vector<double> > > pointWithAngleConstraints;
};


#endif //IKALGORITHMS_BTKPOINTREADADAPTER_H
