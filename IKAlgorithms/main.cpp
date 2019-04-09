#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <vector>
#include <string>
#include <iostream>
#include <tr1/regex>
#include "FabrikSolve.h"
#include "AcquisitionChain.h"

btk::Acquisition::Pointer readAcquisition(const std::string &filename) {
    btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
    reader->SetFilename(filename);
    reader->Update();
    return reader->GetOutput();
};

void writeAcquisition(const btk::Acquisition::Pointer acq, const std::string &filename) {
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

void printPoints(const btk::Acquisition::Pointer acq) {

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

void setTargetAndApplySolver(FabrikSolve &fabrikSolve, AcquisitionChain chain,
                             std::vector<std::pair<int, btk::Point::Pointer> > inputPoints, int index) {

    fabrikSolve.setTarget(chain.pointAt(inputPoints.rbegin()->second, index));
    fabrikSolve.solve();
}

void prepareOutput(btk::Acquisition::Pointer acq, AcquisitionChain &acquisitionChain,
                   std::vector<std::pair<int, btk::Point::Pointer> > &pointPicks,
                   std::vector<std::pair<int, btk::Point::Pointer> > &outputPoints) {
    size_t index = 0;
    btk::Point::Pointer point;

    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator pointsIt = pointPicks.begin();
         pointsIt != pointPicks.end(); ++pointsIt) {
        std::cout << "Data: " << pointsIt->first << " " << pointsIt->second->GetLabel() << " " << index
                  << std::endl;

        point = pointsIt->second->Clone();
        point->SetLabel(pointsIt->second->GetLabel() + " Fabrik");
        acquisitionChain.addToMap(pointsIt->second->GetLabel(), point, true, "L" + index);
        outputPoints.push_back(std::make_pair(pointsIt->first, point));
        index++;
    }
}

void prepareOutputWithConstraints(btk::Acquisition::Pointer acq, AcquisitionChain &acquisitionChain,
                                  std::vector<std::pair<int, btk::Point::Pointer> > &pointPicks,
                                  std::map<int, std::vector<double> > pointConstraintsMap,
                                  std::vector<std::pair<int, btk::Point::Pointer> > &outputPoints) {
    size_t index = 0;
    btk::Point::Pointer point;

    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator pointsIt = pointPicks.begin();
         pointsIt != pointPicks.end(); ++pointsIt) {
        std::cout << "Data: " << pointsIt->first << " " << pointsIt->second->GetLabel() << " " << index
                  << std::endl;

        point = pointsIt->second->Clone();
        point->SetLabel(pointsIt->second->GetLabel() + " Fabrik");
        acquisitionChain.addPointWithConstraints(
                std::make_pair(pointsIt->second, pointConstraintsMap.at(pointsIt->first)));
        acquisitionChain.addToMap(pointsIt->second->GetLabel(), point, true, "L" + index);
        outputPoints.push_back(std::make_pair(pointsIt->first, point));
        index++;
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

void
setOutputDataForBothLegsOnAcquisition(int index, bool areLegsOfSameJointAmounts,
                                      std::vector<std::pair<int, btk::Point::Pointer> > leftOutputPoints,
                                      FabrikSolve fabrikSolveLeft,
                                      std::vector<std::pair<int, btk::Point::Pointer> > rightOutputPoints,
                                      FabrikSolve fabrikSolveRight,
                                      btk::Acquisition::Pointer &printAcquisition) {
    if (areLegsOfSameJointAmounts) {
        for (size_t j = 0; j < leftOutputPoints.size(); ++j) {

            leftOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveLeft.getJoints().at(j).coeff(0),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(1),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(2));
            rightOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveRight.getJoints().at(j).coeff(0),
                                                         fabrikSolveRight.getJoints().at(j).coeff(1),
                                                         fabrikSolveRight.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(rightOutputPoints.at(j).first, rightOutputPoints.at(j).second);
            printAcquisition->SetPoint(leftOutputPoints.at(j).first, leftOutputPoints.at(j).second);
        }
    } else {
        for (size_t j = 0; j < leftOutputPoints.size(); ++j) {

            leftOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveLeft.getJoints().at(j).coeff(0),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(1),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(leftOutputPoints.at(j).first, leftOutputPoints.at(j).second);
        }

        for (size_t j = 0; j < rightOutputPoints.size(); ++j) {

            rightOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveRight.getJoints().at(j).coeff(0),
                                                         fabrikSolveRight.getJoints().at(j).coeff(1),
                                                         fabrikSolveRight.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(rightOutputPoints.at(j).first, rightOutputPoints.at(j).second);
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

std::vector<std::pair<int, btk::Point::Pointer> >
pickFromAcq(const btk::Acquisition::Pointer acq) {
    std::vector<std::pair<int, btk::Point::Pointer> > idsAndPoints;
    std::vector<int> choices;

    int index = 0;
    for (btk::Acquisition::PointIterator acqIt = acq->BeginPoint(); acqIt != acq->EndPoint(); ++acqIt) {
        std::cout << "[" << index << "] " << "(Point: " << acqIt->get()->GetLabel() << "Type: "
                  << acqIt->get()->GetType() << ")" << std::endl;
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
        std::cout << "Added " << choice << " to the chain" << std::endl;
        choices.push_back(choice);
    }

    index = 0;
    std::vector<double> constraintsVector;
    for (btk::Acquisition::PointIterator acqIt = acq->BeginPoint(); acqIt != acq->EndPoint(); ++acqIt) {

        for (std::vector<int>::iterator idChoicesIt = choices.begin(); idChoicesIt != choices.end(); ++idChoicesIt) {
            if (*idChoicesIt == index) {
                std::cout << "Added " << *idChoicesIt << " to " << acqIt->get()->GetLabel() << " to the map"
                          << std::endl;
                idsAndPoints.push_back(std::make_pair(*idChoicesIt, acqIt->get()->Clone()));
            }
        }

        index++;
    }

    return idsAndPoints;
}

std::map<int, std::vector<double> >
pickConstraintsForVector(std::vector<std::pair<int, btk::Point::Pointer> > idPointsPairsVector) {
    std::map<int, std::vector<double> > choiceConstraintsMap;

    double angleConstraints[4] = {90, 90, 90, 90};
    std::vector<double> constraintsVector;
    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator idPointPairsVectorIterator = idPointsPairsVector.begin();
         idPointPairsVectorIterator != idPointsPairsVector.end(); ++idPointPairsVectorIterator) {
        std::cout << "Enter upper angle constraint value for " << idPointPairsVectorIterator->second->GetLabel()
                  << " : "
                  << std::endl;
        std::cin >> angleConstraints[0];
        std::cout << "Enter lower angle constraint value for " << idPointPairsVectorIterator->second->GetLabel()
                  << " : "
                  << std::endl;
        std::cin >> angleConstraints[1];
        std::cout << "Enter left angle constraint value for " << idPointPairsVectorIterator->second->GetLabel() << " : "
                  << std::endl;
        std::cin >> angleConstraints[2];
        std::cout << "Enter right angle constraint value for " << idPointPairsVectorIterator->second->GetLabel()
                  << " : "
                  << std::endl;
        std::cin >> angleConstraints[3];
        std::cout << "Up: " << angleConstraints[0] << "°\nDown: " << angleConstraints[1] << "°\nLeft: "
                  << angleConstraints[2] << "°\nRight: " << angleConstraints[3] << "°" << std::endl;
        constraintsVector = std::vector<double>(angleConstraints, angleConstraints + 4);
        choiceConstraintsMap[idPointPairsVectorIterator->first] = constraintsVector;
    }

    return choiceConstraintsMap;
}

std::vector<std::pair<int, btk::Point::Pointer> >
testDataFromAcqBtk(btk::Acquisition::Pointer acq, std::vector<int> choices) {
    int index = 0;
    std::vector<std::pair<int, btk::Point::Pointer> > pointsStrs;

    for (btk::Acquisition::PointIterator peter = acq->BeginPoint(); peter != acq->EndPoint(); ++peter) {

        for (std::vector<int>::iterator intChoiceIt = choices.begin(); intChoiceIt != choices.end(); ++intChoiceIt) {
            if (*intChoiceIt == index) {
                std::cout << "Added " << *intChoiceIt << " to " << peter->get()->GetLabel() << " to the map"
                          << std::endl;
                pointsStrs.push_back(std::make_pair(*intChoiceIt, peter->get()->Clone()));
            }
        }

        index++;
    }

    return pointsStrs;
}

std::map<int, std::vector<double> >
testConstraintsData(std::vector<std::pair<int, btk::Point::Pointer> > idsPointsPairsVector) {
    std::map<int, std::vector<double> > idAnglesMap;
    double angleConstraintsArray[4] = {30, 30, 2, 2};
    std::vector<double> angleConstraints(angleConstraintsArray, angleConstraintsArray + 4);
    float index = 0.0;
    std::cout << "Size of Point pairs: " << idsPointsPairsVector.size() << std::endl;

    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator idsPointsPairsIterator = idsPointsPairsVector.begin();
         idsPointsPairsIterator != idsPointsPairsVector.end(); ++idsPointsPairsIterator) {
        std::cout << "Index for Constraints " << index << std::endl;
        angleConstraints.at(0) += index;
        angleConstraints.at(1) += index;
        angleConstraints.at(2) += index;
        angleConstraints.at(3) += index;
        idAnglesMap[idsPointsPairsIterator->first] = angleConstraints;
        index += 1.0;
    }
    return idAnglesMap;
}

btk::Point::Pointer createPointOriginBetweenTwoPoints(btk::Acquisition::Pointer acq, btk::Point::Pointer leftPoint,
                                                      btk::Point::Pointer rightPoint, std::string originIdentifierStr) {

    Eigen::Vector3d leftOrigin(leftPoint->GetValues().coeff(0, 0),
                               leftPoint->GetValues().coeff(0, 1),
                               leftPoint->GetValues().coeff(0, 2));
    Eigen::Vector3d rightOrigin(rightPoint->GetValues().coeff(0, 0),
                                rightPoint->GetValues().coeff(0, 1),
                                rightPoint->GetValues().coeff(0, 2));
    Eigen::Vector3d originInbetween = (leftOrigin + rightOrigin) / 2;

    btk::Point::Pointer originPoint = btk::Point::New("Origin" + originIdentifierStr, acq->GetPointFrameNumber());
    for (int i = 0; i < acq->GetPointFrameNumber(); ++i) {
        originPoint->SetDataSlice(i, originInbetween.coeff(0), originInbetween.coeff(1),
                                  originInbetween.coeff(2));
    }

    return originPoint;
}


btk::Acquisition::Pointer writeIkUnconstrainedWithOrigin(const btk::Acquisition::Pointer acq, bool isTestRun = false) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    int testPicksLeft[4] = {0, 4, 10, 11};
    std::vector<int> testPicksLeftVec(testPicksLeft, testPicksLeft + 4);
    std::map<int, std::vector<double> > empty;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks =
            isTestRun ? testDataFromAcqBtk(acq, testPicksLeftVec) : pickFromAcq(acq);


    std::cout << "Choice right" << std::endl;
    int testPicksRight[4] = {1, 12, 18, 19};
    std::vector<int> testDataPicksRightVec(testPicksRight, testPicksRight + 4);
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks =
            isTestRun ? testDataFromAcqBtk(acq, testDataPicksRightVec) : pickFromAcq(acq);

    btk::Point::Pointer originLeftPoint = createPointOriginBetweenTwoPoints(acq, rightPointPicks.begin()->second,
                                                                            leftPointPicks.begin()->second, "Left");
    btk::Point::Pointer originRightPoint = createPointOriginBetweenTwoPoints(acq, rightPointPicks.begin()->second,
                                                                             leftPointPicks.begin()->second, "Right");

    // Create Right Input points vector
    std::vector<std::pair<int, btk::Point::Pointer> > rightInputPoints;
    rightInputPoints.push_back(std::make_pair(20, originRightPoint));
    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator rightPicksIt = rightPointPicks.begin();
         rightPicksIt != rightPointPicks.end(); ++rightPicksIt) {
        rightInputPoints.push_back(std::make_pair(rightPicksIt->first, rightPicksIt->second));
    }

    // Create Left Input points vector
    std::vector<std::pair<int, btk::Point::Pointer> > leftInputPoints;
    leftInputPoints.push_back(std::make_pair(21, originLeftPoint));
    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator leftPicksIt = leftPointPicks.begin();
         leftPicksIt != leftPointPicks.end(); ++leftPicksIt) {
        leftInputPoints.push_back(std::make_pair(leftPicksIt->first, leftPicksIt->second));
    }

    // Create output points
    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    printAcquisition->AppendPoint(originLeftPoint);
    printAcquisition->AppendPoint(originRightPoint);

    std::vector<std::pair<int, btk::Point::Pointer> > leftOutputPoints;
    std::vector<std::pair<int, btk::Point::Pointer> > rightOutputPoints;

    prepareOutput(acq, leftChain, leftPointPicks, leftOutputPoints);
    prepareOutput(acq, rightChain, rightPointPicks, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve fabrikSolveRight(rightChain, tolerance, false);
    FabrikSolve fabrikSolveLeft(leftChain, tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {

        setTargetAndApplySolver(fabrikSolveRight, rightChain, rightInputPoints, index);
        setTargetAndApplySolver(fabrikSolveLeft, leftChain, leftInputPoints, index);
        setOutputDataForBothLegsOnAcquisition(index, rightOutputPoints.size() == leftOutputPoints.size(),
                                              leftOutputPoints, fabrikSolveLeft, rightOutputPoints, fabrikSolveRight,
                                              printAcquisition);
    }
    return printAcquisition;
}


btk::Acquisition::Pointer writeIkUnconstrained(const btk::Acquisition::Pointer acq, bool isTestRun = false) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    int testPicksLeft[4] = {0, 4, 10, 11};
    std::vector<int> testPicksLeftVec(testPicksLeft, testPicksLeft + 4);
    std::vector<std::pair<int, btk::Point::Pointer> > leftInputPoints =
            isTestRun ? testDataFromAcqBtk(acq, testPicksLeftVec) : pickFromAcq(acq);


    std::cout << "Choice right" << std::endl;
    int testPicksRight[4] = {1, 12, 18, 19};
    std::vector<int> testDataPicksRightVec(testPicksRight, testPicksRight + 4);
    std::vector<std::pair<int, btk::Point::Pointer> > rightInputPoints =
            isTestRun ? testDataFromAcqBtk(acq, testDataPicksRightVec) : pickFromAcq(acq);

    // Create output points
    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    std::vector<std::pair<int, btk::Point::Pointer> > leftOutputPoints;
    std::vector<std::pair<int, btk::Point::Pointer> > rightOutputPoints;

    prepareOutput(acq, leftChain, leftInputPoints, leftOutputPoints);
    prepareOutput(acq, rightChain, rightInputPoints, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve fabrikSolveRight(rightChain, tolerance, false);
    FabrikSolve fabrikSolveLeft(leftChain, tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        setTargetAndApplySolver(fabrikSolveRight, rightChain, rightInputPoints, index);
        setTargetAndApplySolver(fabrikSolveLeft, leftChain, leftInputPoints, index);
        setOutputDataForBothLegsOnAcquisition(index,
                                              rightOutputPoints.size() == leftOutputPoints.size(),
                                              leftOutputPoints, fabrikSolveLeft, rightOutputPoints,
                                              fabrikSolveRight,
                                              printAcquisition);

    }
    return printAcquisition;
}


btk::Acquisition::Pointer writeIkConstrained(const btk::Acquisition::Pointer acq, bool isTestRun = false) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    // Hard coded values for testdata frontal Jumping jacks
    int testPicksLeft[4] = {0, 4, 10, 11};
    std::vector<int> testPicksLeftVec(testPicksLeft, testPicksLeft + 4);
    std::map<int, std::vector<double> > jointIdsAngleConstraintsLeftMap;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks =
            isTestRun ? testDataFromAcqBtk(acq, testPicksLeftVec) : pickFromAcq(acq);
    jointIdsAngleConstraintsLeftMap = isTestRun ? testConstraintsData(leftPointPicks) : pickConstraintsForVector(
            leftPointPicks);


    std::cout << "Choice right" << std::endl;
    // Hard coded values for testdata frontal Jumping jacks
    int testPicksRight[4] = {1, 12, 18, 19};
    std::vector<int> testDataPicksRightVec(testPicksRight, testPicksRight + 4);
    std::map<int, std::vector<double> > jointIdsAngleConstraintsRightMap;
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks =
            isTestRun ? testDataFromAcqBtk(acq, testDataPicksRightVec) : pickFromAcq(acq);
    jointIdsAngleConstraintsRightMap = isTestRun ? testConstraintsData(rightPointPicks) : (pickConstraintsForVector(
            rightPointPicks));


    // Create Right Input points vector
    std::vector<std::pair<int, btk::Point::Pointer> > rightInputPoints;
    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator rightPicksIt = rightPointPicks.begin();
         rightPicksIt != rightPointPicks.end(); ++rightPicksIt) {
        rightInputPoints.push_back(std::make_pair(rightPicksIt->first, rightPicksIt->second));
    }

    // Create Left Input points vector
    std::vector<std::pair<int, btk::Point::Pointer> > leftInputPoints;
    for (std::vector<std::pair<int, btk::Point::Pointer> >::iterator leftPicksIt = leftPointPicks.begin();
         leftPicksIt != leftPointPicks.end(); ++leftPicksIt) {
        leftInputPoints.push_back(std::make_pair(leftPicksIt->first, leftPicksIt->second));
    }

    // Create output points
    btk::Acquisition::Pointer printAcquisition = acq->Clone();
    std::vector<std::pair<int, btk::Point::Pointer> > leftOutputPoints;
    std::vector<std::pair<int, btk::Point::Pointer> > rightOutputPoints;
    prepareOutputWithConstraints(acq, leftChain, leftPointPicks, jointIdsAngleConstraintsLeftMap, leftOutputPoints);
    prepareOutputWithConstraints(acq, rightChain, rightPointPicks, jointIdsAngleConstraintsRightMap, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve
            fabrikSolveRight(rightChain, tolerance, true);
    FabrikSolve
            fabrikSolveLeft(leftChain, tolerance, true);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(rightChain.pointAt(rightInputPoints.rbegin()->second, index));
        fabrikSolveRight.solve();

        fabrikSolveLeft.setTarget(leftChain.pointAt(leftInputPoints.rbegin()->second, index));
        fabrikSolveLeft.solve();

        setOutputDataForBothLegsOnAcquisition(index,
                                              rightOutputPoints.size() == leftOutputPoints.size(),
                                              leftOutputPoints, fabrikSolveLeft, rightOutputPoints,
                                              fabrikSolveRight,
                                              printAcquisition);
    }
    return printAcquisition;
}

int main(int argc, char **argv) {

    std::string optionalArguments = "";
    std::string unconstrainedStr("-u");
    std::string constrainedStr("-c");
    std::string testRegexStr("-t");
    std::string staticMiddleRegexStr("-s");
    std::string c3dFormatStr(".c3d");

    for (int i = 1; i < argc; ++i) {
        optionalArguments.append(argv[i]);
    }
    std::string inputStr = "";
    std::string outputStr = "";
    if (argc > 2) {
        inputStr.append(argv[1]);
        outputStr.append(argv[2]);
    }

    if (argc >= 3 && inputStr.find(c3dFormatStr) != std::string::npos &&
        outputStr.find(c3dFormatStr) != std::string::npos) {
        btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
        if (optionalArguments.find(unconstrainedStr) != std::string::npos) {
            if (optionalArguments.find(staticMiddleRegexStr) != std::string::npos) {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "Static middle unconstrained test" << std::endl;
                    btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrainedWithOrigin(acq, true);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "Static middle unconstrained test complete" << std::endl;
                } else {
                    std::cout << "Static middle unconstrained" << std::endl;
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrainedWithOrigin(acq);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "Static middle unconstrained complete" << std::endl;
                }
            } else {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "unconstrained test" << std::endl;
                    btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq, true);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "unconstrained test complete" << std::endl;
                } else {
                    std::cout << "unconstrained" << std::endl;
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "unconstrained complete" << std::endl;
                }
            }
        } else if (optionalArguments.find(constrainedStr) != std::string::npos) {
            if (optionalArguments.find(staticMiddleRegexStr) != std::string::npos) {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "[WIP] Static middle constrained test" << std::endl;
                    btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, true);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "[WIP] Static middle constrained complete" << std::endl;
                } else {
                    std::cout << "[WIP] Static middle constrained" << std::endl;
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "[WIP] Static middle constrained complete" << std::endl;
                }
            } else {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "Test constraint " << std::endl;
                    btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, true);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "Test constraint complete" << std::endl;
                } else {
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "Just constrained" << std::endl;
                }
            }
        } else if (optionalArguments.find(testRegexStr) != std::string::npos) {
            std::cout << "Test regex" << std::endl;
            std::cout << "unconstrained test" << std::endl;
            btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
            btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq, true);
            writeAcquisition(ikAcq, argv[2]);
            std::cout << "unconstrained test complete" << std::endl;
        } else {
            std::cout
                    << "usage: ./IKAlgorithms <inputC3D> <outputC3D> [-s for origin middle] [-u for unconstrained xor -c for constrained] [-t for test data]"
                    << std::endl << std::endl;
        }
    } else {
        std::cout
                << "usage: ./IKAlgorithms <inputC3D> <outputC3D> [-s for origin middle] [-u for unconstrained xor -c for constrained] [-t for test data]"
                << std::endl << std::endl;
    }
    return 0;
};
