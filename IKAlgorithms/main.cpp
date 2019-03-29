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
#include "AcqPointReadAdapter.h"

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

std::vector<Eigen::Vector3d> getSecondsVector(std::vector<std::pair<std::string, Eigen::Vector3d> > pairs) {
    std::vector<Eigen::Vector3d> vectors;
    for (size_t index = 0; index < pairs.size(); ++index) {
        vectors.push_back(pairs.at(index).second);
    }
    return vectors;
}

std::vector<std::pair<std::string, Eigen::Vector3d> >
setVectorInPair(std::vector<std::pair<std::string, Eigen::Vector3d> > pairs, std::vector<Eigen::Vector3d> vectors,
                bool isZeroDoubleOrigin) {
    for (size_t index = 0; index < vectors.size(); ++index) {
        if (!(isZeroDoubleOrigin && index == 0))
            pairs.at(index) = std::make_pair(pairs.at(index).first, vectors.at(index));
    }
    return pairs;
}

std::pair<std::vector<std::pair<std::string, float> >, float> calculateDistances(std::vector<Eigen::Vector3d> joints,
                                                                                 std::vector<std::pair<std::string, Eigen::Vector3d> > debugVectors) {

    std::vector<std::pair<std::string, float> > distances;
    float sumOfAllLenghts;

    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            distances.push_back(std::make_pair(
                    debugVectors.at(index + 1).first + debugVectors.at(index).first,
                    (joints.at(index + 1) - joints.at(index)).norm()));
            std::cout << "distanceR: " << distances.at(index).first << ": " << distances.at(index).second
                      << std::endl;
            sumOfAllLenghts += (joints.at(index + 1) - joints.at(index)).norm();
        }
        std::cout << debugVectors.at(index).first << ": ["
                  << joints.at(index).coeff(0) << ","
                  << joints.at(index).coeff(1) << ","
                  << joints.at(index).coeff(2) << "]"
                  << std::endl;
    }
    return std::make_pair(distances, sumOfAllLenghts);
}

std::pair<std::vector<float>, float> calculateDistancesDebugless(std::vector<Eigen::Vector3d> joints) {

    std::vector<float> distances;
    float sumOfAllLenghts;

    for (size_t index = 0; index < joints.size(); ++index) {
        if (index + 1 < joints.size()) {
            distances.push_back((joints.at(index + 1) - joints.at(index)).norm());
            sumOfAllLenghts += (joints.at(index + 1) - joints.at(index)).norm();
        }
    }
    return std::make_pair(distances, sumOfAllLenghts);
}

std::map<int, std::string> pickFromAcq(btk::Acquisition::Pointer acq) {
    std::map<int, std::string> pointStrs;
    std::vector<int> choices;

    int index = 0;
    for (btk::Acquisition::PointIterator peter = acq->BeginPoint(); peter != acq->EndPoint(); ++peter) {
        std::cout << "[" << index << "] " << "(Point: " << peter->get()->GetLabel() << "Type: "
                  << peter->get()->GetType() << ")" << std::endl;
        index++;
    }

    int choice = 0;
    std::cout << "Pick Points by Number to build chain (pick -1 to complete your choice, repetitions not possible)"
              << std::endl;

    while (true) {
        std::cout << "> " ;
        std::cin >> choice;
        if(choice == -1) break;
        std::cout << "Added " << choice << " to the memory" << std::endl;
        choices.push_back(choice);
    }

    index = 0;

    for (btk::Acquisition::PointIterator peter = acq->BeginPoint(); peter != acq->EndPoint(); ++peter) {

        for (std::vector<int>::iterator intChoiceIt = choices.begin(); intChoiceIt != choices.end(); ++intChoiceIt) {
            if (*intChoiceIt == index) {
                std::cout << "Added " << *intChoiceIt << " to " << peter->get()->GetLabel() <<" to the map" << std::endl;
                pointStrs.insert(std::make_pair(*intChoiceIt, peter->get()->GetLabel()));
            }
        }

        index++;
    }

    return pointStrs;
}


btk::Acquisition::Pointer writeIkUnconstrained(btk::Acquisition::Pointer acq) {
    AcqPointReadAdapter acqPointReadAdapterRight;
    AcqPointReadAdapter acqPointReadAdapterLeft;
    float tolerance = 0.00001f;
    std::pair<std::string, Eigen::Vector3d> targetL;
    std::pair<std::string, Eigen::Vector3d> originL;
    std::pair<std::string, Eigen::Vector3d> targetR;
    std::pair<std::string, Eigen::Vector3d> originR;
    std::vector<float> distancesLeft;
    std::vector<float> distancesRight;
    float sumOfAllLengthsLeft;
    float sumOfAllLengthsRight;
    bool constrained = false;
    int constrLeft = 89;
    int constrRight = 89;
    int constrUp = 89;
    int constrDown = 89;

    std::map<int, std::string> pointStrs = pickFromAcq(acq);

    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    btk::Point::Pointer waistPointL = btk::Point::New("Skeleton_002:WaistLFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointL = btk::Point::New("Skeleton_002:LKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointL = btk::Point::New("Skeleton_002:LHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointL = btk::Point::New("Skeleton_002:LToeTip2", acq->GetPointFrameNumber());


    btk::Point::Pointer waistPointR = btk::Point::New("Skeleton_002:WaistRFront2", acq->GetPointFrameNumber());
    btk::Point::Pointer kneePointR = btk::Point::New("Skeleton_002:RKneeOut2", acq->GetPointFrameNumber());
    btk::Point::Pointer heelPointR = btk::Point::New("Skeleton_002:RHeel2", acq->GetPointFrameNumber());
    btk::Point::Pointer tipToePointR = btk::Point::New("Skeleton_002:RToeTip2", acq->GetPointFrameNumber());


    acqPointReadAdapterRight.addToMap("Skeleton_002:WaistRFront", acq->GetPoint("Skeleton_002:WaistRFront"), true,
                                      "R1");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RKneeOut", acq->GetPoint("Skeleton_002:RKneeOut"), true, "R2");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RHeel", acq->GetPoint("Skeleton_002:RHeel"), true, "R3");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RToeTip", acq->GetPoint("Skeleton_002:RToeTip"), true, "R4");

    acqPointReadAdapterLeft.addToMap("Skeleton_002:WaistLFront", acq->GetPoint("Skeleton_002:WaistLFront"), true, "L1");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LKneeOut", acq->GetPoint("Skeleton_002:LKneeOut"), true, "L2");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LHeel", acq->GetPoint("Skeleton_002:LHeel"), true, "L3");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LToeTip", acq->GetPoint("Skeleton_002:LToeTip"), true, "L4");


    originR = std::make_pair("OR", acqPointReadAdapterRight.pointAt("Skeleton_002:WaistRFront", 0));
    originL = std::make_pair("OL", acqPointReadAdapterLeft.pointAt("Skeleton_002:WaistLFront", 0));

    targetR = std::make_pair("TR", acqPointReadAdapterRight.pointAt("Skeleton_002:RToeTip", 1));
    targetL = std::make_pair("TL", acqPointReadAdapterLeft.pointAt("Skeleton_002:LToeTip", 1));

    std::vector<Eigen::Vector3d> jointsRight = acqPointReadAdapterRight.getPositionsPerFrame(0);
    std::vector<Eigen::Vector3d> jointsLeft = acqPointReadAdapterLeft.getPositionsPerFrame(0);

    jointsRight.resize(3);
    jointsLeft.resize(3);

    std::pair<std::vector<float>, float> distancesRightPair = calculateDistancesDebugless(jointsRight);

    distancesRight = distancesRightPair.first;
    sumOfAllLengthsRight = distancesRightPair.second;

    std::pair<std::vector<float>, float> distancesLeftPair = calculateDistancesDebugless(jointsLeft);
    distancesLeft = distancesLeftPair.first;
    sumOfAllLengthsLeft = distancesLeftPair.second;


    FabrikSolve fabrikSolveRight(jointsRight, targetR.second, originR.second, sumOfAllLengthsRight,
                                 distancesRight, tolerance, false);
    FabrikSolve fabrikSolveLeft(jointsLeft, targetL.second, originL.second, sumOfAllLengthsLeft,
                                distancesLeft, tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(acqPointReadAdapterRight.pointAt("Skeleton_002:RToeTip", index));
        fabrikSolveRight.solve();
        jointsRight = fabrikSolveRight.getJoints();

        fabrikSolveLeft.setTarget(acqPointReadAdapterLeft.pointAt("Skeleton_002:LToeTip", index));
        fabrikSolveLeft.solve();
        jointsLeft = fabrikSolveLeft.getJoints();


        waistPointR->SetDataSlice(index, jointsRight.at(0).coeff(0), jointsRight.at(0).coeff(1),
                                  jointsRight.at(0).coeff(2));
        kneePointR->SetDataSlice(index, jointsRight.at(1).coeff(0), jointsRight.at(1).coeff(1),
                                 jointsRight.at(1).coeff(2));
        heelPointR->SetDataSlice(index, jointsRight.at(2).coeff(0), jointsRight.at(2).coeff(1),
                                 jointsRight.at(2).coeff(2));

        waistPointL->SetDataSlice(index, jointsLeft.at(0).coeff(0), jointsLeft.at(0).coeff(1),
                                  jointsLeft.at(0).coeff(2));
        kneePointL->SetDataSlice(index, jointsLeft.at(1).coeff(0), jointsLeft.at(1).coeff(1),
                                 jointsLeft.at(1).coeff(2));
        heelPointL->SetDataSlice(index, jointsLeft.at(2).coeff(0), jointsLeft.at(2).coeff(1),
                                 jointsLeft.at(2).coeff(2));

        printAcquisition->SetPoint(1, waistPointR);
        printAcquisition->SetPoint(12, kneePointR);
        printAcquisition->SetPoint(18, heelPointR);

        printAcquisition->SetPoint(0, waistPointL);
        printAcquisition->SetPoint(4, kneePointL);
        printAcquisition->SetPoint(10, heelPointL);
    }
    return printAcquisition;
}

btk::Acquisition::Pointer writeIkConstrained(btk::Acquisition::Pointer acq) {
    AcqPointReadAdapter acqPointReadAdapterRight;
    AcqPointReadAdapter acqPointReadAdapterLeft;
    float tolerance = 0.00001f;
    std::pair<std::string, Eigen::Vector3d> targetL;
    std::pair<std::string, Eigen::Vector3d> originL;
    std::pair<std::string, Eigen::Vector3d> targetR;
    std::pair<std::string, Eigen::Vector3d> originR;
    std::vector<std::pair<std::string, float> > distancesLeft;
    std::vector<std::pair<std::string, float> > distancesRight;
    float sumOfAllLengthsLeft;
    float sumOfAllLengthsRight;
    bool constrained = true;
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


    acqPointReadAdapterRight.addToMap("Skeleton_002:WaistRFront", acq->GetPoint("Skeleton_002:WaistRFront"), true,
                                      "R1");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RKneeOut", acq->GetPoint("Skeleton_002:RKneeOut"), true, "R2");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RHeel", acq->GetPoint("Skeleton_002:RHeel"), true, "R3");
    acqPointReadAdapterRight.addToMap("Skeleton_002:RToeTip", acq->GetPoint("Skeleton_002:RToeTip"), true, "R4");

    acqPointReadAdapterLeft.addToMap("Skeleton_002:WaistLFront", acq->GetPoint("Skeleton_002:WaistLFront"), true, "L1");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LKneeOut", acq->GetPoint("Skeleton_002:LKneeOut"), true, "L2");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LHeel", acq->GetPoint("Skeleton_002:LHeel"), true, "L3");
    acqPointReadAdapterLeft.addToMap("Skeleton_002:LToeTip", acq->GetPoint("Skeleton_002:LToeTip"), true, "L4");


    originR = std::make_pair("OR", acqPointReadAdapterRight.pointAt("Skeleton_002:WaistRFront", 0));
    originL = std::make_pair("OL", acqPointReadAdapterLeft.pointAt("Skeleton_002:WaistLFront", 0));

    targetR = std::make_pair("TR", acqPointReadAdapterRight.pointAt("Skeleton_002:RToeTip", 0));
    targetL = std::make_pair("TL", acqPointReadAdapterLeft.pointAt("Skeleton_002:LToeTip", 0));

    std::vector<Eigen::Vector3d> jointsRight = acqPointReadAdapterRight.getPositionsPerFrame(0);
    std::vector<Eigen::Vector3d> jointsLeft = acqPointReadAdapterLeft.getPositionsPerFrame(0);

    std::pair<std::vector<std::pair<std::string, float> >, float> distancesRightPair = calculateDistances(jointsRight,
                                                                                                          acqPointReadAdapterRight.getDebuggablePairsVectorOfFrame(
                                                                                                                  0));
    distancesRight = distancesRightPair.first;
    sumOfAllLengthsRight = distancesRightPair.second;

    std::pair<std::vector<std::pair<std::string, float> >, float> distancesLeftPair = calculateDistances(jointsLeft,
                                                                                                         acqPointReadAdapterLeft.getDebuggablePairsVectorOfFrame(
                                                                                                                 0));
    distancesLeft = distancesLeftPair.first;
    sumOfAllLengthsLeft = distancesLeftPair.second;


    FabrikSolve fabrikSolveRight(acqPointReadAdapterRight.getPositionsPerFrame(0), targetR.second, originR.second,
                                 sumOfAllLengthsRight,
                                 getSecondsFloat(distancesRight), tolerance, true);
    FabrikSolve fabrikSolveLeft(acqPointReadAdapterLeft.getPositionsPerFrame(0), targetL.second, originL.second,
                                sumOfAllLengthsLeft,
                                getSecondsFloat(distancesLeft), tolerance, true);
    fabrikSolveLeft.solve();
    fabrikSolveRight.solve();

    std::cout << "Sum: " << std::endl;

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 0; index < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(acqPointReadAdapterRight.pointAt("Skeleton_002:RToeTip", index));
        fabrikSolveRight.solve();


        fabrikSolveLeft.setTarget(acqPointReadAdapterLeft.pointAt("Skeleton_002:LToeTip", index));
        fabrikSolveLeft.solve();


        waistPointR->SetDataSlice(index, jointsRight.at(0).coeff(0), jointsRight.at(0).coeff(1),
                                  jointsRight.at(0).coeff(2));
        kneePointR->SetDataSlice(index, jointsRight.at(1).coeff(0), jointsRight.at(1).coeff(1),
                                 jointsRight.at(1).coeff(2));
        heelPointR->SetDataSlice(index, jointsRight.at(2).coeff(0), jointsRight.at(2).coeff(1),
                                 jointsRight.at(2).coeff(2));

        waistPointL->SetDataSlice(index, jointsLeft.at(0).coeff(0), jointsLeft.at(0).coeff(1),
                                  jointsLeft.at(0).coeff(2));
        kneePointL->SetDataSlice(index, jointsLeft.at(1).coeff(0), jointsLeft.at(1).coeff(1),
                                 jointsLeft.at(1).coeff(2));
        heelPointL->SetDataSlice(index, jointsLeft.at(2).coeff(0), jointsLeft.at(2).coeff(1),
                                 jointsLeft.at(2).coeff(2));

        printAcquisition->SetPoint(1, waistPointR);
        printAcquisition->SetPoint(12, kneePointR);
        printAcquisition->SetPoint(18, heelPointR);

        printAcquisition->SetPoint(0, waistPointL);
        printAcquisition->SetPoint(4, kneePointL);
        printAcquisition->SetPoint(10, heelPointL);
    }
    return printAcquisition;
    return printAcquisition;
}

int main(int argc, char **argv) {
    btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
    std::cout << argv[3] << std::endl;

    if (argv[3] != NULL && std::strcmp(argv[3], "unconstrained") == 0) {
        btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq);
        writeAcquisition(ikAcq, argv[2]);
    } else {
        btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq);
        writeAcquisition(ikAcq, argv[2]);
    }

    return 0;
};
