#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <vector>
#include <string>
#include <iostream>
#include <typeinfo>
#include <cmath>
#include "FabrikSolve.h"

btk::Acquisition::Pointer readAcquisition(const std::string &filename) {
    btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(filename);
    reader->Update();
    return reader->GetOutput();
};

void writeAcquisition(btk::Acquisition::Pointer acq, const std::string &filename) {
    btk::AcquisitionFileWriter::Pointer writer = btk::AcquisitionFileWriter::New();
    writer->SetInput(acq);
    writer->SetFilename(filename);
    writer->Update();
};

btk::Acquisition::Pointer readC3d(const std::string &filename) {
    btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
    btk::C3DFileIO::Pointer io = btk::C3DFileIO::New();
    reader->SetAcquisitionIO(io);
    reader->SetFilename(filename);
    reader->Update();
}

std::string GetPointTypeAsString(btk::Point::Type t) {
    if (t == btk::Point::Marker)
        return "Marker";
    else if (t == btk::Point::Angle)
        return "Angle";
    else if (t == btk::Point::Force)
        return "Force";
    else if (t == btk::Point::Moment)
        return "Moment";
    else if (t == btk::Point::Power)
        return "Power";
    else if (t == btk::Point::Scalar)
        return "Scalar";
}

void printPoints(btk::Acquisition::Pointer acq) {

    btk::PointCollection::Pointer marker_pts = acq->GetPoints();

    if (marker_pts->GetItemNumber() != 0) {
        for (size_t f = 0;
             f < marker_pts->GetFrontItem()->GetFrameNumber();
             ++f) {
            for (btk::Acquisition::PointIterator it = marker_pts->Begin();
                 it != marker_pts->End();
                 ++it) {
                btk::Point::Pointer pt = *it;
                std::cout << "[" << f << "] " << pt->GetLabel() << " " << "x: " << pt->GetValues().coeff(f, 0) << " y: "
                          << pt->GetValues().coeff(f, 1) << " z: " << pt->GetValues().coeff(f, 2) << std::endl;
            }
            std::cout << std::endl;
        }
    }
}

void printPointsWithLabel(btk::Acquisition::Pointer acq, std::string labelStr) {
    btk::Point::Pointer itP = acq->GetPoint(labelStr);

    if (acq->GetPointFrameNumber() != 0) {
        for (size_t f = 0;
             f < acq->GetPointFrameNumber();
             ++f) {
            std::cout << "[" << f << "] " << labelStr << " " << "x: " << itP->GetValues().coeff(f, 0) << " y: "
                      << itP->GetValues().coeff(f, 1) << " z: " << itP->GetValues().coeff(f, 2) << std::endl;
        }
    }
}

Eigen::Vector3f pointAt(btk::Point::Pointer itP, int frameNumber) {
    Eigen::Vector3f point(itP->GetValues().coeff(frameNumber, 0),
                          itP->GetValues().coeff(frameNumber, 1),
                          itP->GetValues().coeff(frameNumber, 2));
    return point;
}

float sum(std::vector<float> summands) {
    float sum;
    for (size_t index = 0; index < summands.size(); ++index) {
        sum += summands.at(index);
    }
    return sum;
}


std::vector<float> getSecondsFloat(std::vector<std::pair<std::string, float> > pairs) {
    std::vector<float> floats;
    for (size_t index = 0; index < pairs.size(); ++index) {
        floats.push_back(pairs.at(index).second);
    }
    return floats;
}

std::vector<Eigen::Vector3f> getSecondsVector(std::vector<std::pair<std::string, Eigen::Vector3f> > pairs) {
    std::vector<Eigen::Vector3f> vectors;
    for (size_t index = 0; index < pairs.size(); ++index) {
        vectors.push_back(pairs.at(index).second);
    }
    return vectors;
}

std::vector<std::pair<std::string, Eigen::Vector3f> > setVectorInPair(std::vector<std::pair<std::string, Eigen::Vector3f> > pairs, std::vector<Eigen::Vector3f> vectors) {
    for(size_t index = 0; index < vectors.size(); ++index){
        pairs.at(index) = std::make_pair(pairs.at(index).first, vectors.at(index));
    }
    return pairs;
}


btk::Acquisition::Pointer writeIk(btk::Acquisition::Pointer acq) {
    std::vector<std::pair<std::string, Eigen::Vector3f> > joints;
    float tolerance = 0.001f;
    std::pair<std::string, Eigen::Vector3f> target;
    std::pair<std::string, Eigen::Vector3f> origin;
    std::vector<std::pair<std::string, float> > distances;
    float sumOfAllLengths;
    bool constrained = false;
    int constrLeft = 89;
    int constrRight = 89;
    int constrUp = 89;
    int constrDown = 89;

    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    btk::Point::Pointer waistPoint = btk::Point::New("Skeleton_001:WaistLFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePoint = btk::Point::New("Skeleton_001:LKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPoint = btk::Point::New("Skeleton_001:LHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePoint = btk::Point::New("Skeleton_001:LToeTip2", acq->GetPointFrameNumber());


    Eigen::Vector3f waist = pointAt(acq->GetPoint("Skeleton_001:WaistLFront"), 0);
    Eigen::Vector3f knee = pointAt(acq->GetPoint("Skeleton_001:LKneeOut"), 0);
    Eigen::Vector3f heel = pointAt(acq->GetPoint("Skeleton_001:LHeel"), 0);
    Eigen::Vector3f tipToe = pointAt(acq->GetPoint("Skeleton_001:LToeTip"), 0);

    joints.push_back(std::make_pair("p1", Eigen::Vector3f(waist.coeff(0), waist.coeff(1), waist.coeff(2))));
    joints.push_back(std::make_pair("p2", Eigen::Vector3f(knee.coeff(0), knee.coeff(1), knee.coeff(2))));
    joints.push_back(std::make_pair("p3", Eigen::Vector3f(heel.coeff(0), heel.coeff(1), heel.coeff(2))));
    origin = std::make_pair("O", Eigen::Vector3f(waist.coeff(0), waist.coeff(1), waist.coeff(2)));
    target = std::make_pair("T", Eigen::Vector3f(tipToe.coeff(0), tipToe.coeff(1), tipToe.coeff(2)));

    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            distances.push_back(std::make_pair(joints.at(index + 1).first + joints.at(index).first,
                                               (joints.at(index + 1).second - joints.at(index).second).norm()));
            std::cout << "distance: " << distances.at(index).first << ": " << distances.at(index).second << std::endl;
            sumOfAllLengths += (joints.at(index + 1).second - joints.at(index).second).norm();
        }
        std::cout << joints.at(index).first << ": [" << joints.at(index).second.coeff(0) << "," << joints.at(index).second.coeff(1)<< "," << joints.at(index).second.coeff(2) << "]" << std::endl;
    }

    FabrikSolve fabrikSolve(getSecondsVector(joints), target.second, origin.second, sumOfAllLengths,
                            getSecondsFloat(distances), tolerance);
    fabrikSolve.solve();

    joints = setVectorInPair(joints, fabrikSolve.getJoints());
    std::cout << "Sum: " << std::endl;

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 0; index < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;
        tipToe = pointAt(acq->GetPoint("Skeleton_001:LToeTip"), index);
        fabrikSolve.setTarget(tipToe);
        fabrikSolve.solve();
        joints = setVectorInPair(joints, fabrikSolve.getJoints());

        waistPoint->SetDataSlice(index, joints.at(0).second.coeff(0), joints.at(0).second.coeff(1), joints.at(0).second.coeff(2));
        kneePoint->SetDataSlice(index, joints.at(1).second.coeff(0), joints.at(1).second.coeff(1), joints.at(1).second.coeff(2));
        heelPoint->SetDataSlice(index, joints.at(2).second.coeff(0), joints.at(2).second.coeff(1), joints.at(2).second.coeff(2));

        printAcquisition->SetPoint(0, waistPoint);
        printAcquisition->SetPoint(4, kneePoint);
        printAcquisition->SetPoint(10, heelPoint);
    }
    return printAcquisition;
}

int main(int argc, char **argv) {
    btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
    btk::Acquisition::Pointer ikAcq = writeIk(acq);
    writeAcquisition(ikAcq, argv[2]);
    return 0;
};
