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

std::vector<std::pair<std::string, Eigen::Vector3f> >
setVectorInPair(std::vector<std::pair<std::string, Eigen::Vector3f> > pairs, std::vector<Eigen::Vector3f> vectors,
                bool isZeroDoubleOrigin) {
    for (size_t index = 0; index < vectors.size(); ++index) {
        if (!(isZeroDoubleOrigin && index == 0))
            pairs.at(index) = std::make_pair(pairs.at(index).first, vectors.at(index));
    }
    return pairs;
}


btk::Acquisition::Pointer writeIk(btk::Acquisition::Pointer acq) {
    std::vector<std::pair<std::string, Eigen::Vector3f> > jointsLeft;
    std::vector<std::pair<std::string, Eigen::Vector3f> > jointsRight;
    float tolerance = 0.00001f;
    std::pair<std::string, Eigen::Vector3f> targetL;
    std::pair<std::string, Eigen::Vector3f> originL;
    std::pair<std::string, Eigen::Vector3f> targetR;
    std::pair<std::string, Eigen::Vector3f> originR;
    std::vector<std::pair<std::string, float> > distancesLeft;
    std::vector<std::pair<std::string, float> > distancesRight;
    float sumOfAllLengthsLeft;
    float sumOfAllLengthsRight;
    bool constrained = false;
    int constrLeft = 89;
    int constrRight = 89;
    int constrUp = 89;
    int constrDown = 89;

    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    btk::Point::Pointer waistPointL = btk::Point::New("Skeleton_002:WaistLFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointL = btk::Point::New("Skeleton_002:LKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointL = btk::Point::New("Skeleton_002:LHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointL = btk::Point::New("Skeleton_002:LToeTip2", acq->GetPointFrameNumber());

    btk::Point::Pointer waistPointR = btk::Point::New("Skeleton_002:WaistRFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointR = btk::Point::New("Skeleton_002:RKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointR = btk::Point::New("Skeleton_002:RHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointR = btk::Point::New("Skeleton_002:RToeTip2", acq->GetPointFrameNumber());


    Eigen::Vector3f waistR = pointAt(acq->GetPoint("Skeleton_002:WaistRFront"), 0);
    Eigen::Vector3f kneeR = pointAt(acq->GetPoint("Skeleton_002:RKneeOut"), 0);
    Eigen::Vector3f heelR = pointAt(acq->GetPoint("Skeleton_002:RHeel"), 0);
    Eigen::Vector3f tipToeR = pointAt(acq->GetPoint("Skeleton_002:RToeTip"), 0);

    Eigen::Vector3f waistL = pointAt(acq->GetPoint("Skeleton_002:WaistLFront"), 0);
    Eigen::Vector3f kneeL = pointAt(acq->GetPoint("Skeleton_002:LKneeOut"), 0);
    Eigen::Vector3f heelL = pointAt(acq->GetPoint("Skeleton_002:LHeel"), 0);
    Eigen::Vector3f tipToeL = pointAt(acq->GetPoint("Skeleton_002:LToeTip"), 0);

    jointsLeft.push_back(std::make_pair("L1", Eigen::Vector3f(waistL.coeff(0), waistL.coeff(1), waistL.coeff(2))));
    jointsLeft.push_back(std::make_pair("L2", Eigen::Vector3f(kneeL.coeff(0), kneeL.coeff(1), kneeL.coeff(2))));
    jointsLeft.push_back(std::make_pair("L3", Eigen::Vector3f(heelL.coeff(0), heelL.coeff(1), heelL.coeff(2))));

    jointsRight.push_back(std::make_pair("R1", Eigen::Vector3f(waistR.coeff(0), waistR.coeff(1), waistR.coeff(2))));
    jointsRight.push_back(std::make_pair("R1", Eigen::Vector3f(kneeR.coeff(0), kneeR.coeff(1), kneeR.coeff(2))));
    jointsRight.push_back(std::make_pair("R1", Eigen::Vector3f(heelR.coeff(0), heelR.coeff(1), heelR.coeff(2))));

    originR = std::make_pair("OR", Eigen::Vector3f(waistR.coeff(0), waistR.coeff(1), waistR.coeff(2)));
    originL = std::make_pair("OL", Eigen::Vector3f(waistL.coeff(0), waistL.coeff(1), waistL.coeff(2)));
    targetR = std::make_pair("TR", Eigen::Vector3f(tipToeR.coeff(0), tipToeR.coeff(1), tipToeR.coeff(2)));
    targetL = std::make_pair("TL", Eigen::Vector3f(tipToeL.coeff(0), tipToeL.coeff(1), tipToeL.coeff(2)));

    for (size_t index = 0; index < jointsRight.size(); ++index) {
        if (index + 1 < jointsRight.size()) {
            distancesRight.push_back(std::make_pair(jointsRight.at(index + 1).first + jointsRight.at(index).first,
                                                    (jointsRight.at(index + 1).second -
                                                     jointsRight.at(index).second).norm()));
            std::cout << "distanceR: " << distancesRight.at(index).first << ": " << distancesRight.at(index).second
                      << std::endl;
            sumOfAllLengthsRight += (jointsRight.at(index + 1).second - jointsRight.at(index).second).norm();
        }
        std::cout << jointsLeft.at(index).first << ": [" << jointsLeft.at(index).second.coeff(0) << ","
                  << jointsLeft.at(index).second.coeff(1) << "," << jointsLeft.at(index).second.coeff(2) << "]"
                  << std::endl;
    }

    for (size_t index = 0; index < jointsLeft.size(); ++index) {
        if (index + 1 < jointsLeft.size()) {
            distancesLeft.push_back(std::make_pair(jointsLeft.at(index + 1).first + jointsLeft.at(index).first,
                                                   (jointsLeft.at(index + 1).second -
                                                    jointsLeft.at(index).second).norm()));
            std::cout << "distanceL: " << distancesLeft.at(index).first << ": " << distancesLeft.at(index).second
                      << std::endl;
            sumOfAllLengthsLeft += (jointsLeft.at(index + 1).second - jointsLeft.at(index).second).norm();
        }
        std::cout << jointsLeft.at(index).first << ": [" << jointsLeft.at(index).second.coeff(0) << ","
                  << jointsLeft.at(index).second.coeff(1) << "," << jointsLeft.at(index).second.coeff(2) << "]"
                  << std::endl;
    }

    FabrikSolve fabrikSolveRight(getSecondsVector(jointsRight), targetR.second, originR.second, sumOfAllLengthsRight,
                                 getSecondsFloat(distancesRight), tolerance);
    FabrikSolve fabrikSolveLeft(getSecondsVector(jointsLeft), targetL.second, originL.second, sumOfAllLengthsLeft,
                                getSecondsFloat(distancesLeft), tolerance);
    fabrikSolveLeft.solve();
    fabrikSolveRight.solve();

    jointsLeft = setVectorInPair(jointsLeft, fabrikSolveLeft.getJoints(), false);
    jointsRight = setVectorInPair(jointsRight, fabrikSolveRight.getJoints(), false);
    std::cout << "Sum: " << std::endl;

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 0; index < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;
        tipToeR = pointAt(acq->GetPoint("Skeleton_002:RToeTip"), index);
        tipToeL = pointAt(acq->GetPoint("Skeleton_002:LToeTip"), index);

        fabrikSolveRight.setTarget(tipToeR);
        fabrikSolveRight.solve();
        jointsRight = setVectorInPair(jointsRight, fabrikSolveRight.getJoints(), false);

        fabrikSolveLeft.setTarget(tipToeL);
        fabrikSolveLeft.solve();
        jointsLeft = setVectorInPair(jointsLeft, fabrikSolveLeft.getJoints(), false);

        waistPointR->SetDataSlice(index, jointsRight.at(0).second.coeff(0), jointsRight.at(0).second.coeff(1),
                                  jointsRight.at(0).second.coeff(2));
        kneePointR->SetDataSlice(index, jointsRight.at(1).second.coeff(0), jointsRight.at(1).second.coeff(1),
                                 jointsRight.at(1).second.coeff(2));
        heelPointR->SetDataSlice(index, jointsRight.at(2).second.coeff(0), jointsRight.at(2).second.coeff(1),
                                 jointsRight.at(2).second.coeff(2));

        waistPointL->SetDataSlice(index, jointsLeft.at(0).second.coeff(0), jointsLeft.at(0).second.coeff(1),
                                  jointsLeft.at(0).second.coeff(2));
        kneePointL->SetDataSlice(index, jointsLeft.at(1).second.coeff(0), jointsLeft.at(1).second.coeff(1),
                                 jointsLeft.at(1).second.coeff(2));
        heelPointL->SetDataSlice(index, jointsLeft.at(2).second.coeff(0), jointsLeft.at(2).second.coeff(1),
                                 jointsLeft.at(2).second.coeff(2));

        printAcquisition->SetPoint(1, waistPointR);
        printAcquisition->SetPoint(12, kneePointR);
        printAcquisition->SetPoint(18, heelPointR);

        printAcquisition->SetPoint(0, waistPointL);
        printAcquisition->SetPoint(4, kneePointL);
        printAcquisition->SetPoint(10, heelPointL);
    }
    return printAcquisition;
}


btk::Acquisition::Pointer writeOneChain(btk::Acquisition::Pointer acq) {
    std::vector<std::pair<std::string, Eigen::Vector3f> > jointsLeft;
    std::vector<std::pair<std::string, Eigen::Vector3f> > jointsRight;
    float tolerance = 0.001f;
    std::pair<std::string, Eigen::Vector3f> targetL;
    std::pair<std::string, Eigen::Vector3f> origin;
    std::pair<std::string, Eigen::Vector3f> targetR;
    std::vector<std::pair<std::string, float> > distancesLeft;
    std::vector<std::pair<std::string, float> > distancesRight;
    float sumOfAllLengthsLeft;
    float sumOfAllLengthsRight;
    bool constrained = false;
    int constrLeft = 89;
    int constrRight = 89;
    int constrUp = 89;
    int constrDown = 89;

    btk::Acquisition::Pointer printAcquisition = acq->Clone();

    // Data Acquisition Left Leg
    btk::Point::Pointer waistPointL = btk::Point::New("Skeleton_002:WaistLFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointL = btk::Point::New("Skeleton_002:LKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointL = btk::Point::New("Skeleton_002:LHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointL = btk::Point::New("Skeleton_002:LToeTip2", acq->GetPointFrameNumber());

    // Data Acquisition Right Leg
    btk::Point::Pointer waistPointR = btk::Point::New("Skeleton_002:WaistRFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointR = btk::Point::New("Skeleton_002:RKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointR = btk::Point::New("Skeleton_002:RHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointR = btk::Point::New("Skeleton_002:RToeTip2", acq->GetPointFrameNumber());

    // Prepare Data writing Right Leg
    Eigen::Vector3f waistR = pointAt(acq->GetPoint("Skeleton_002:WaistRFront"), 0);
    Eigen::Vector3f kneeR = pointAt(acq->GetPoint("Skeleton_002:RKneeOut"), 0);
    Eigen::Vector3f heelR = pointAt(acq->GetPoint("Skeleton_002:RHeel"), 0);
    Eigen::Vector3f tipToeR = pointAt(acq->GetPoint("Skeleton_002:RToeTip"), 0);

    Eigen::Vector3f waistMiddle = (pointAt(acq->GetPoint("Skeleton_002:WaistRFront"), 0) +
                                   pointAt(acq->GetPoint("Skeleton_002:WaistLFront"), 0)) / 2;

    // Prepare Data writing Left Leg
    Eigen::Vector3f waistL = pointAt(acq->GetPoint("Skeleton_002:WaistLFront"), 0);
    Eigen::Vector3f kneeL = pointAt(acq->GetPoint("Skeleton_002:LKneeOut"), 0);
    Eigen::Vector3f heelL = pointAt(acq->GetPoint("Skeleton_002:LHeel"), 0);
    Eigen::Vector3f tipToeL = pointAt(acq->GetPoint("Skeleton_002:LToeTip"), 0);

    // Fill joint Data into Debuggable Vector Pair container
    jointsLeft.push_back(
            std::make_pair("0", Eigen::Vector3f(waistMiddle.coeff(0), waistMiddle.coeff(1), waistMiddle.coeff(2))));
    jointsLeft.push_back(std::make_pair("L1", Eigen::Vector3f(waistL.coeff(0), waistL.coeff(1), waistL.coeff(2))));
    jointsLeft.push_back(std::make_pair("L2", Eigen::Vector3f(kneeL.coeff(0), kneeL.coeff(1), kneeL.coeff(2))));
    jointsLeft.push_back(std::make_pair("L3", Eigen::Vector3f(heelL.coeff(0), heelL.coeff(1), heelL.coeff(2))));
    jointsLeft.push_back(std::make_pair("L4", Eigen::Vector3f(tipToeL.coeff(0), tipToeL.coeff(1), tipToeL.coeff(2))));

    jointsRight.push_back(
            std::make_pair("0", Eigen::Vector3f(waistMiddle.coeff(0), waistMiddle.coeff(1), waistMiddle.coeff(2))));
    jointsRight.push_back(std::make_pair("R1", Eigen::Vector3f(waistR.coeff(0), waistR.coeff(1), waistR.coeff(2))));
    jointsRight.push_back(std::make_pair("R2", Eigen::Vector3f(kneeR.coeff(0), kneeR.coeff(1), kneeR.coeff(2))));
    jointsRight.push_back(std::make_pair("R3", Eigen::Vector3f(heelR.coeff(0), heelR.coeff(1), heelR.coeff(2))));
    jointsRight.push_back(std::make_pair("R4", Eigen::Vector3f(tipToeR.coeff(0), tipToeR.coeff(1), tipToeR.coeff(2))));

    // Set Debuggable Target and Origin
    origin = std::make_pair("O", Eigen::Vector3f(waistMiddle.coeff(0), waistMiddle.coeff(1), waistMiddle.coeff(2)));
    targetR = std::make_pair("TR", Eigen::Vector3f(tipToeR.coeff(0), tipToeR.coeff(1), tipToeR.coeff(2)));
    targetL = std::make_pair("TL", Eigen::Vector3f(tipToeL.coeff(0), tipToeL.coeff(1), tipToeL.coeff(2)));

    // Calculate Distances of right leg and Debug
    for (size_t index = 0; index < jointsRight.size(); ++index) {
        if (index + 1 < jointsRight.size()) {
            distancesRight.push_back(std::make_pair(jointsRight.at(index + 1).first + jointsRight.at(index).first,
                                                    (jointsRight.at(index + 1).second -
                                                     jointsRight.at(index).second).norm()));
            std::cout << "distanceR: " << distancesRight.at(index).first << ": " << distancesRight.at(index).second
                      << std::endl;
            sumOfAllLengthsRight += (jointsRight.at(index + 1).second - jointsRight.at(index).second).norm();
        }
        std::cout << jointsRight.at(index).first << ": [" << jointsRight.at(index).second.coeff(0) << ","
                  << jointsRight.at(index).second.coeff(1) << "," << jointsRight.at(index).second.coeff(2) << "]"
                  << std::endl;
    }

    // Calculate Distances of left leg and Debug
    for (size_t index = 0; index < jointsLeft.size(); ++index) {
        if (index + 1 < jointsLeft.size()) {
            distancesLeft.push_back(std::make_pair(jointsLeft.at(index + 1).first + jointsLeft.at(index).first,
                                                   (jointsLeft.at(index + 1).second -
                                                    jointsLeft.at(index).second).norm()));
            std::cout << "distanceL: " << distancesLeft.at(index).first << ": " << distancesLeft.at(index).second
                      << std::endl;
            sumOfAllLengthsLeft += (jointsLeft.at(index + 1).second - jointsLeft.at(index).second).norm();
        }
        std::cout << jointsLeft.at(index).first << ": [" << jointsLeft.at(index).second.coeff(0) << ","
                  << jointsLeft.at(index).second.coeff(1) << "," << jointsLeft.at(index).second.coeff(2) << "]"
                  << std::endl;
    }

    // Create Solver
    FabrikSolve fabrikSolveRight(getSecondsVector(jointsRight), targetR.second, origin.second, sumOfAllLengthsRight,
                                 getSecondsFloat(distancesRight), tolerance);
    FabrikSolve fabrikSolveLeft(getSecondsVector(jointsLeft), targetL.second, origin.second, sumOfAllLengthsLeft,
                                getSecondsFloat(distancesLeft), tolerance);

    // Loop through the Data
    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 0; index < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        // Get Tiptoe Point
        tipToeR = pointAt(acq->GetPoint("Skeleton_002:RToeTip"), index);
        tipToeL = pointAt(acq->GetPoint("Skeleton_002:LToeTip"), index);

        // Set Target and solve right
        fabrikSolveRight.setTarget(tipToeR);
        fabrikSolveRight.solve();
        jointsRight = setVectorInPair(jointsRight, fabrikSolveRight.getJoints(), true);

        // Set Taget and solve left
        fabrikSolveLeft.setTarget(tipToeL);
        fabrikSolveLeft.solve();
        jointsLeft = setVectorInPair(jointsLeft, fabrikSolveLeft.getJoints(), true);

        // Set Data for right acquisition
        waistPointR->SetDataSlice(index, jointsRight.at(1).second.coeff(0), jointsRight.at(1).second.coeff(1),
                                  jointsRight.at(1).second.coeff(2));
        kneePointR->SetDataSlice(index, jointsRight.at(2).second.coeff(0), jointsRight.at(2).second.coeff(1),
                                 jointsRight.at(2).second.coeff(2));
        heelPointR->SetDataSlice(index, jointsRight.at(3).second.coeff(0), jointsRight.at(3).second.coeff(1),
                                 jointsRight.at(3).second.coeff(2));

        // Set Data for left acquisition
        waistPointL->SetDataSlice(index, jointsLeft.at(1).second.coeff(0), jointsLeft.at(1).second.coeff(1),
                                  jointsLeft.at(1).second.coeff(2));
        kneePointL->SetDataSlice(index, jointsLeft.at(2).second.coeff(0), jointsLeft.at(2).second.coeff(1),
                                 jointsLeft.at(2).second.coeff(2));
        heelPointL->SetDataSlice(index, jointsLeft.at(3).second.coeff(0), jointsLeft.at(3).second.coeff(1),
                                 jointsLeft.at(3).second.coeff(2));

    }
    // Set
    printAcquisition->SetPoint(1, waistPointR);
    printAcquisition->SetPoint(12, kneePointR);
    printAcquisition->SetPoint(18, heelPointR);

    printAcquisition->SetPoint(0, waistPointL);
    printAcquisition->SetPoint(4, kneePointL);
    printAcquisition->SetPoint(10, heelPointL);
    return printAcquisition;
}

int main(int argc, char **argv) {
    btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
    btk::Acquisition::Pointer ikAcq = writeOneChain(acq);
    writeAcquisition(ikAcq, argv[2]);
    return 0;
};
