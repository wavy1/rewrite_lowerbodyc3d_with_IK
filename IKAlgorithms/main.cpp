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
#include "AcquisitionChain.h"

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
    std::cout << "The first pick is the origin, the last pick is the target" << std::endl;

    while (true) {
        std::cout << "> ";
        std::cin >> choice;
        if (choice == -1) break;
        std::cout << "Added " << choice << " to the memory" << std::endl;
        choices.push_back(choice);
    }

    index = 0;

    for (btk::Acquisition::PointIterator peter = acq->BeginPoint(); peter != acq->EndPoint(); ++peter) {

        for (std::vector<int>::iterator intChoiceIt = choices.begin(); intChoiceIt != choices.end(); ++intChoiceIt) {
            if (*intChoiceIt == index) {
                std::cout << "Added " << *intChoiceIt << " to " << peter->get()->GetLabel() << " to the map"
                          << std::endl;
                pointStrs.insert(std::make_pair(*intChoiceIt, peter->get()->GetLabel()));
            }
        }

        index++;
    }

    return pointStrs;
}


std::map<int, std::string> testDataFromAcq(btk::Acquisition::Pointer acq, std::vector<int> choices) {
    int index = 0;
    std::map<int, std::string> pointStrs;

    for (btk::Acquisition::PointIterator peter = acq->BeginPoint(); peter != acq->EndPoint(); ++peter) {

        for (std::vector<int>::iterator intChoiceIt = choices.begin(); intChoiceIt != choices.end(); ++intChoiceIt) {
            if (*intChoiceIt == index) {
                std::cout << "Added " << *intChoiceIt << " to " << peter->get()->GetLabel() << " to the map"
                          << std::endl;
                pointStrs.insert(std::make_pair(*intChoiceIt, peter->get()->GetLabel()));
            }
        }

        index++;
    }

    return pointStrs;
}


btk::Acquisition::Pointer writeIkUnconstrained(btk::Acquisition::Pointer acq) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;


    std::cout << "Choice left" << std::endl;
    std::map<int, std::string> pointIdToLabelLeft = pickFromAcq(acq);
    std::cout << "Choice right" << std::endl;
    std::map<int, std::string> pointIdToLabelRight = pickFromAcq(acq);

    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    std::vector<btk::Point::Pointer> leftOutputPoints;
    std::vector<btk::Point::Pointer> rightOutputPoints;
    char index = 0;


    for (std::map<int, std::string>::iterator mapLeftIt = pointIdToLabelLeft.begin();
         mapLeftIt != pointIdToLabelLeft.end(); ++mapLeftIt) {
        leftOutputPoints.push_back(btk::Point::New(mapLeftIt->second + "Fabrik", acq->GetPointFrameNumber()));
        leftChain.addToMap(mapLeftIt->second, acq->GetPoint(mapLeftIt->second), true, "L" + index);
        index++;
    }

    std::vector<Eigen::Vector3d> jointsLeft;
    jointsLeft = leftChain.getPositionsPerFrame(0);
    jointsLeft.resize(jointsLeft.size() - 1);
    leftChain.calculateDistancesDebugless(jointsLeft);

    index = 0;
    for (std::map<int, std::string>::iterator mapRightIt = pointIdToLabelRight.begin();
         mapRightIt != pointIdToLabelRight.end(); ++mapRightIt) {
        rightOutputPoints.push_back(btk::Point::New(mapRightIt->second + "Fabrik", acq->GetPointFrameNumber()));
        rightChain.addToMap(mapRightIt->second, acq->GetPoint(mapRightIt->second), true, "R" + index);
        index++;
    }
    std::vector<Eigen::Vector3d> jointsRight = rightChain.getPositionsPerFrame(0);
    jointsRight.resize(jointsRight.size() - 1);
    rightChain.calculateDistancesDebugless(jointsRight);


    FabrikSolve fabrikSolveRight(jointsRight, rightChain.pointAt(pointIdToLabelRight.rbegin()->second, 1),
                                 jointsRight.at(0),
                                 rightChain.getSetSumOfAllLenghts(),
                                 rightChain.getDistances(), tolerance, false);
    FabrikSolve fabrikSolveLeft(jointsLeft, leftChain.pointAt(pointIdToLabelLeft.rbegin()->second, 1),
                                jointsLeft.at(0),
                                leftChain.getSetSumOfAllLenghts(),
                                leftChain.getDistances(), tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setOrigin(rightChain.pointAt(pointIdToLabelRight.begin()->second, index));
        fabrikSolveLeft.setOrigin(leftChain.pointAt(pointIdToLabelLeft.begin()->second, index));
        fabrikSolveRight.setTarget(rightChain.pointAt(pointIdToLabelRight.rbegin()->second, index));
        fabrikSolveRight.solve();
        jointsRight = fabrikSolveRight.getJoints();

        fabrikSolveLeft.setTarget(leftChain.pointAt(pointIdToLabelLeft.rbegin()->second, index));
        fabrikSolveLeft.solve();
        jointsLeft = fabrikSolveLeft.getJoints();

        size_t j = 0;

        std::map<int, std::string>::iterator pointToStringsLeft = pointIdToLabelLeft.begin();
        std::map<int, std::string>::iterator pointToStringsRight = pointIdToLabelRight.begin();
        while (j < jointsLeft.size()) {
            leftOutputPoints.at(j).get()->SetDataSlice(index, jointsLeft.at(j).coeff(0), jointsLeft.at(j).coeff(1),
                                                       jointsLeft.at(j).coeff(2));
            rightOutputPoints.at(j).get()->SetDataSlice(index, jointsRight.at(j).coeff(0), jointsRight.at(j).coeff(1),
                                                        jointsRight.at(j).coeff(2));
            printAcquisition->SetPoint(pointToStringsRight->first, rightOutputPoints.at(j));
            printAcquisition->SetPoint(pointToStringsLeft->first, leftOutputPoints.at(j));
            ++j;
            ++pointToStringsLeft;
            ++pointToStringsRight;
        }

    }
    return printAcquisition;
}

btk::Acquisition::Pointer writeIkConstrained(btk::Acquisition::Pointer acq) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.01f;

    std::cout << "Choice left" << std::endl;
    std::map<int, std::string> pointIdToLabelLeft = pickFromAcq(acq);
    std::cout << "Choice right" << std::endl;
    std::map<int, std::string> pointIdToLabelRight = pickFromAcq(acq);

    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    std::vector<btk::Point::Pointer> leftOutputPoints;
    std::vector<btk::Point::Pointer> rightOutputPoints;
    char index = 0;

    for (std::map<int, std::string>::iterator mapLeftIt = pointIdToLabelLeft.begin();
         mapLeftIt != pointIdToLabelLeft.end(); ++mapLeftIt) {
        leftOutputPoints.push_back(btk::Point::New(mapLeftIt->second + "Fabrik", acq->GetPointFrameNumber()));
        leftChain.addToMap(mapLeftIt->second, acq->GetPoint(mapLeftIt->second), true, "L" + index);
        index++;
    }
    std::vector<Eigen::Vector3d> jointsLeft = leftChain.getPositionsPerFrame(0);
    jointsLeft.resize(3);
    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));

    index = 0;
    for (std::map<int, std::string>::iterator mapRightIt = pointIdToLabelRight.begin();
         mapRightIt != pointIdToLabelRight.end(); ++mapRightIt) {
        rightOutputPoints.push_back(btk::Point::New(mapRightIt->second + "Fabrik", acq->GetPointFrameNumber()));
        rightChain.addToMap(mapRightIt->second, acq->GetPoint(mapRightIt->second), true, "R" + index);
        index++;
    }
    std::vector<Eigen::Vector3d> jointsRight = rightChain.getPositionsPerFrame(0);
    jointsRight.resize(3);
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));


    FabrikSolve fabrikSolveRight(jointsRight, rightChain.pointAt(pointIdToLabelRight.rbegin()->second, 1),
                                 rightChain.pointAt(pointIdToLabelRight.begin()->second, 0),
                                 rightChain.getSetSumOfAllLenghts(),
                                 rightChain.getDistances(), tolerance, true);
    fabrikSolveRight.setAllConeConstraints(45, 45, 45, 45);
    FabrikSolve fabrikSolveLeft(jointsLeft, leftChain.pointAt(pointIdToLabelLeft.rbegin()->second, 1),
                                leftChain.pointAt(pointIdToLabelLeft.begin()->second, 0),
                                leftChain.getSetSumOfAllLenghts(),
                                leftChain.getDistances(), tolerance, true);
    fabrikSolveLeft.setAllConeConstraints(45, 45, 45, 45);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(rightChain.pointAt(pointIdToLabelRight.rbegin()->second, index));
        fabrikSolveRight.solve();
        jointsRight = fabrikSolveRight.getJoints();

        fabrikSolveLeft.setTarget(leftChain.pointAt(pointIdToLabelLeft.rbegin()->second, index));
        fabrikSolveLeft.solve();
        jointsLeft = fabrikSolveLeft.getJoints();

        size_t j = 0;
        std::map<int, std::string>::iterator pointToStringsLeft = pointIdToLabelLeft.begin();
        std::map<int, std::string>::iterator pointToStringsRight = pointIdToLabelRight.begin();
        while (j < jointsLeft.size()) {
            leftOutputPoints.at(j).get()->SetDataSlice(index, jointsLeft.at(j).coeff(0), jointsLeft.at(j).coeff(1),
                                                       jointsLeft.at(j).coeff(2));
            rightOutputPoints.at(j).get()->SetDataSlice(index, jointsRight.at(j).coeff(0), jointsRight.at(j).coeff(1),
                                                        jointsRight.at(j).coeff(2));
            printAcquisition->SetPoint(pointToStringsRight->first, rightOutputPoints.at(j));
            printAcquisition->SetPoint(pointToStringsLeft->first, leftOutputPoints.at(j));
            ++j;
            ++pointToStringsLeft;
            ++pointToStringsRight;
        }
    }
    return printAcquisition;
}

int main(int argc, char **argv) {
    btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
    std::cout << argc << std::endl;
    std::cout << argv[3] << std::endl;
    std::cout << argv[4] << std::endl;

    if (argc == 4) {
        if (std::strcmp(argv[3], "--unconstrained") == 0 || std::strcmp(argv[3], "-u") == 0) {
            btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq);
            writeAcquisition(ikAcq, argv[2]);
        } else if (std::strcmp(argv[3], "--constrained") == 0 || std::strcmp(argv[3], "-c") == 0) {
            btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq);
            writeAcquisition(ikAcq, argv[2]);
        }
    } else if (argc == 5) {
        if ((std::strcmp(argv[3], "--unconstrained") == 0 && std::strcmp(argv[4], "--static-origin") == 0) ||
            (std::strcmp(argv[4], "--unconstrained") == 0 && std::strcmp(argv[3], "--static-origin") == 0) ||
            (std::strcmp(argv[3], "-u") == 0 && std::strcmp(argv[4], "-s") == 0) ||
            (std::strcmp(argv[4], "-u") == 0 && std::strcmp(argv[3], "-s") == 0)) {
            std::cout << "Writing with static middle between individiual origins" << std::endl;
            btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq);
            writeAcquisition(ikAcq, argv[2]);
        }
    }

    return 0;
};
