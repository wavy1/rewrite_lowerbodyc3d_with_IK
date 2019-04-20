#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <tr1/regex>
#include <sstream>
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


btk::Acquisition::Pointer
writeIkUnconstrained(const btk::Acquisition::Pointer acq, std::vector<int> commandLinePicksRight,
                     std::vector<int> commandLinePicksLeft, std::string csvFileName = "") {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    std::ofstream distancesCSV;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > leftInputPoints =
            commandLinePicksLeft.size() != 0 ? testDataFromAcqBtk(acq, commandLinePicksLeft) : pickFromAcq(acq);


    std::cout << "Choice right" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > rightInputPoints =
            commandLinePicksRight.size() != 0 ? testDataFromAcqBtk(acq, commandLinePicksRight) : pickFromAcq(acq);

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

    if(csvFileName.length() > 0){
        distancesCSV.open(csvFileName.c_str());
        for(size_t index = 0; index < leftChain.getDistances().size(); ++index){
            distancesCSV << ";L" << index << "-L" << index+1;
        }

        for(size_t index = 0; index < rightChain.getDistances().size(); ++index){
            distancesCSV << ";R" << index << "-R" << index+1;
            if(index + 1 == rightChain.getDistances().size()){
                distancesCSV << "\n";
            }
        }
        distancesCSV.close();
    }

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

        if(std::string(csvFileName).length() > 0) {
            leftChain.writeDistancesIntoCSV(fabrikSolveLeft.getJoints(), csvFileName, index, false);
            rightChain.writeDistancesIntoCSV(fabrikSolveRight.getJoints(), csvFileName, index, true);
        }

    }
    return printAcquisition;
}


btk::Acquisition::Pointer writeIkConstrained(const btk::Acquisition::Pointer acq, std::map<int, std::vector<double> > leftPicksAndConstraints,
                                             std::map<int, std::vector<double> > rightPicksAndConstraints, std::string csvFileName = "") {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    std::ofstream distancesCSV;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    // Hard coded values for testdata frontal Jumping jacks
    std::vector<int> testPicksLeftVec;
    for (std::map<int, std::vector<double> >::iterator leftPicksAndConstraintsIterator = leftPicksAndConstraints.begin();
         leftPicksAndConstraintsIterator != leftPicksAndConstraints.end(); ++leftPicksAndConstraintsIterator) {
        testPicksLeftVec.push_back(leftPicksAndConstraintsIterator->first);
    }


    std::map<int, std::vector<double> > jointIdsAngleConstraintsLeftMap;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks =
            testPicksLeftVec.empty() ? pickFromAcq(acq) : testDataFromAcqBtk(acq, testPicksLeftVec);
    jointIdsAngleConstraintsLeftMap = leftPicksAndConstraints.empty() ? pickConstraintsForVector(
            leftPointPicks) : leftPicksAndConstraints;


    std::cout << "Choice right" << std::endl;
    std::vector<int> testPicksRightVec;
    for (std::map<int, std::vector<double> >::iterator rightPicksAndConstraintsIterator = rightPicksAndConstraints.begin();
         rightPicksAndConstraintsIterator != rightPicksAndConstraints.end(); ++rightPicksAndConstraintsIterator) {
        testPicksRightVec.push_back(rightPicksAndConstraintsIterator->first);
    }
    std::map<int, std::vector<double> > jointIdsAngleConstraintsRightMap;
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks =
            testPicksRightVec.empty() ? pickFromAcq(acq) : testDataFromAcqBtk(acq, testPicksRightVec);
    jointIdsAngleConstraintsRightMap = rightPicksAndConstraints.empty() ? (pickConstraintsForVector(rightPointPicks))
                                                                        : rightPicksAndConstraints;


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

    if(csvFileName.length() > 0){
        distancesCSV.open(csvFileName.c_str());
        for(size_t index = 0; index < leftChain.getDistances().size(); ++index){
            distancesCSV << ";L" << index << "-L" << index+1;
        }

        for(size_t index = 0; index < rightChain.getDistances().size(); ++index){
            distancesCSV << ";R" << index << "-R" << index+1;
            if(index + 1 == rightChain.getDistances().size()){
                distancesCSV << "\n";
            }
        }
        distancesCSV.close();
    }

    FabrikSolve
            fabrikSolveRight(rightChain, tolerance, true);
    FabrikSolve
            fabrikSolveLeft(leftChain, tolerance, true);

    if(std::string(csvFileName).length() > 0) {
        leftChain.writeDistancesIntoCSV(fabrikSolveLeft.getJoints(), csvFileName, 0, false);
        rightChain.writeDistancesIntoCSV(fabrikSolveRight.getJoints(), csvFileName, 0, true);
    }

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (int index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
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

        fabrikSolveLeft.printDistances();
        fabrikSolveRight.printDistances();

        if(std::string(csvFileName).length() > 0) {
            leftChain.writeDistancesIntoCSV(fabrikSolveLeft.getJoints(), csvFileName, index, false);
            rightChain.writeDistancesIntoCSV(fabrikSolveRight.getJoints(), csvFileName, index, true);
        }

    }
    std::cout << csvFileName << std::string(csvFileName).length() << std::endl;
    return printAcquisition;
}

void split(const std::string &s, const std::string &sep, std::vector<std::string> &v) {
    typedef std::string::const_iterator iter;
    iter b = s.begin(), e = s.end(), i;
    iter sep_b = sep.begin(), sep_e = sep.end();

    while (b != e) {
        i = std::search(b, e, sep_b, sep_e);

        if (b == e) {
            v.push_back(std::string(b, e));
            break;
        } else if (i == b) {
            b = i + sep.length();
        } else {
            v.push_back(std::string(b, i));
            b = i;
        }
    }
}

void outputDifferenceCSV(btk::Acquisition::Pointer originalACQ, btk::Acquisition::Pointer modifiedACQ, std::string fileName){
    std::cout << "print delta CSV" << std::endl;
    btk::PointCollection::Pointer originalACQ_points = originalACQ->GetPoints();
    btk::PointCollection::Pointer modifiedACQ_points = modifiedACQ->GetPoints();

    std::ofstream origCSVStream;

    origCSVStream.open(fileName.c_str());
    origCSVStream << "\n;" ;
    if (originalACQ_points->GetItemNumber() != 0) {
        for (btk::Acquisition::PointIterator it = originalACQ_points->Begin(); it != originalACQ_points->End(); ++it) {

            origCSVStream  << it->get()->GetLabel() << ";;;";

        }
        origCSVStream << "\n;";
        for (btk::Acquisition::PointIterator it = originalACQ_points->Begin(); it != originalACQ_points->End(); ++it) {

            origCSVStream << "x;y;z;";

        }
        origCSVStream << "\n";
        for (size_t f = 0; f < originalACQ_points->GetFrontItem()->GetFrameNumber(); ++f) {
            origCSVStream << f << ";";

            for (btk::Acquisition::PointIterator it = originalACQ_points->Begin(); it != originalACQ_points->End(); ++it) {
                btk::Point::Pointer pt = *it;
                origCSVStream << pt->GetValues().coeff(f, 0) << ";" << pt->GetValues().coeff(f, 1) << ";" << pt->GetValues().coeff(f, 2) << ";";
            }
            origCSVStream << "\n";
        }
    }

    origCSVStream << "Modified\n" ;
    if (modifiedACQ_points->GetItemNumber() != 0) {
        for (btk::Acquisition::PointIterator it = modifiedACQ_points->Begin(); it != modifiedACQ_points->End(); ++it) {

            origCSVStream << it->get()->GetLabel() << ";;;";
        }
        origCSVStream << "\n;";
        for (btk::Acquisition::PointIterator it = modifiedACQ_points->Begin(); it != modifiedACQ_points->End(); ++it) {
            origCSVStream << "x;y;z;";
        }
        origCSVStream << "\n";
        for (size_t f = 0; f < modifiedACQ_points->GetFrontItem()->GetFrameNumber(); ++f) {
            origCSVStream << f << ";";

            for (btk::Acquisition::PointIterator it = modifiedACQ_points->Begin(); it != modifiedACQ_points->End(); ++it) {
                btk::Point::Pointer pt = *it;
                origCSVStream << pt->GetValues().coeff(f, 0) << ";" << pt->GetValues().coeff(f, 1) << ";" << pt->GetValues().coeff(f, 2) << ";";
            }
            origCSVStream << "\n";
        }
    }

    origCSVStream.close();
}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

std::vector<int> getCommandLinePick(const std::string &inputString, const std::string &openDelimiter,
                                    const char &delimiterInbetweenNumbers, const std::string &closingDelim) {

    std::size_t foundPos = inputString.find(openDelimiter);
    std::string afterFoundPosToEnd = inputString.substr(foundPos + openDelimiter.size());
    std::size_t endPos = afterFoundPosToEnd.find(closingDelim);
    std::cout << inputString.substr(foundPos + openDelimiter.size(), endPos) << std::endl;
    std::string picksStr = inputString.substr(foundPos + openDelimiter.size(), endPos);


    std::vector<std::string> picksStringVector = split(picksStr, delimiterInbetweenNumbers);
    std::vector<int> picksIntegerVector;
    for (std::vector<std::string>::iterator picksStringVectorIt = picksStringVector.begin();
         picksStringVectorIt != picksStringVector.end(); ++picksStringVectorIt) {
        std::stringstream iss(*picksStringVectorIt);
        int number;
        iss >> number;
        picksIntegerVector.push_back(number);
    }
    return picksIntegerVector;
}

std::map<int, std::vector<double> >
getCommandLinePickAndConstraint(const std::string &inputString, const std::string &openDelimiter,
                                const char &delimiterInbetweenConstraints, const std::string &closingDelim,
                                const char &delimiterBetweenids, const std::string &endOfConstraint,
                                const std::string idToConstraintsPointerString) {
    std::cout << std::endl;
    std::cout << "Input String: " << inputString << std::endl;
    std::cout << "Open delim: " << openDelimiter << std::endl;
    std::cout << "Delimiter in between Constraints: " << delimiterInbetweenConstraints << std::endl;
    std::cout << "Closing Delimiter: " << closingDelim << std::endl;
    std::cout << "Delimiter Between Ids: " << delimiterBetweenids << std::endl;
    std::cout << "End of Constraints: " << endOfConstraint << std::endl;
    std::cout << "Id to Constraints Pointer String: " << idToConstraintsPointerString << std::endl;


    std::map<int, std::vector<double> > picksIntegerVector;
    std::vector<double> constraintsVector;

    std::size_t foundPos = inputString.find(openDelimiter);
    std::string afterFoundPosToEnd = inputString.substr(foundPos + openDelimiter.size());
    std::size_t endPos = afterFoundPosToEnd.find(closingDelim);

    std::cout << inputString.substr(foundPos + openDelimiter.size(), endPos) << std::endl;
    std::string picksStr = inputString.substr(foundPos + openDelimiter.size(), endPos);

    std::vector<std::string> picksConstraintVectorString = split(picksStr, delimiterBetweenids);

    for (std::vector<std::string>::iterator picksConstraintVectorStringIterator = picksConstraintVectorString.begin();
         picksConstraintVectorStringIterator !=
         picksConstraintVectorString.end(); ++picksConstraintVectorStringIterator) {
        foundPos = picksConstraintVectorStringIterator->find(idToConstraintsPointerString);
        std::stringstream iss(picksConstraintVectorStringIterator->substr(0, foundPos));
        int number;
        iss >> number;

        foundPos = picksConstraintVectorStringIterator->find(idToConstraintsPointerString);
        afterFoundPosToEnd = inputString.substr(foundPos + idToConstraintsPointerString.size());
        endPos = afterFoundPosToEnd.find(endOfConstraint);

        picksStr = picksConstraintVectorStringIterator->substr(foundPos + idToConstraintsPointerString.size(), endPos);
        std::vector<std::string> constraintsStr = split(picksStr, delimiterInbetweenConstraints);

        for (std::vector<std::string>::iterator constraintVectorIterator = constraintsStr.begin();
             constraintVectorIterator != constraintsStr.end(); ++constraintVectorIterator) {
            std::stringstream fss(*constraintVectorIterator);
            double angle;
            fss >> angle;
            constraintsVector.push_back(angle);
        }
        picksIntegerVector[number] = constraintsVector;
        constraintsVector.clear();
    }

    return picksIntegerVector;
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
    std::vector<std::string> strings = split(outputStr, '/');
    std::string outputFileName = strings.at(strings.size()-1);
    std::string outputName = (outputFileName.substr(0, outputFileName.length()-4) + "_distances.csv");

    std::cout << "Number args" << argc << std::endl;

    for (size_t i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }

    char jointIdDelimiter = ';';
    char constraintsDelimiter = ',';
    std::string constraintsCloseDelimiter = "]";
    std::string constraintsBeginSequence = "->[";
    std::string closingDelim = "}";
    std::string rightJointsAndConstraintsOpen = "joints-constraints-right={";
    std::string leftJointsAndConstraintsOpen = "joints-constraints-left={";
    std::string leftDelimOpen = "joints-left={";
    std::string rightDelimOpen = "joints-right={";

    std::vector<int> leftPicksInt;
    std::vector<int> rightPicksInt;
    std::map<int, std::vector<double> > leftPicksAndConstraints;
    std::map<int, std::vector<double> > rightPicksAndConstraints;

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
                    rightPicksInt = getCommandLinePick(optionalArguments, rightDelimOpen, jointIdDelimiter,
                                                       closingDelim);
                    leftPicksInt = getCommandLinePick(optionalArguments, leftDelimOpen, jointIdDelimiter,
                                                      closingDelim);
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq, rightPicksInt, leftPicksInt, outputName);
                    writeAcquisition(ikAcq, argv[2]);
                    outputDifferenceCSV(acq, ikAcq, outputFileName + "_delta.csv");
                    std::cout << "unconstrained test complete" << std::endl;
                } else {
                    std::cout << "unconstrained" << std::endl;
                    btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq, rightPicksInt, leftPicksInt, outputName);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "unconstrained complete" << std::endl;
                }
            }
        } else if (optionalArguments.find(constrainedStr) != std::string::npos) {
            if (optionalArguments.find(staticMiddleRegexStr) != std::string::npos) {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "[WIP] Static middle constrained test" << std::endl;
                    leftPicksAndConstraints = getCommandLinePickAndConstraint(
                            optionalArguments, leftJointsAndConstraintsOpen, constraintsDelimiter, closingDelim, jointIdDelimiter,
                            constraintsCloseDelimiter, constraintsBeginSequence);

                    rightPicksAndConstraints = getCommandLinePickAndConstraint(
                            optionalArguments, rightJointsAndConstraintsOpen, constraintsDelimiter, closingDelim, jointIdDelimiter,
                            constraintsCloseDelimiter, constraintsBeginSequence);

                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, leftPicksAndConstraints,
                                                                         rightPicksAndConstraints);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "[WIP] Static middle constrained complete" << std::endl;
                } else {

                    std::cout << "[WIP] Static middle constrained" << std::endl;
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, leftPicksAndConstraints,
                                                                         rightPicksAndConstraints);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "[WIP] Static middle constrained complete" << std::endl;
                }
            } else {
                if (optionalArguments.find(testRegexStr) != std::string::npos) {
                    std::cout << "Test constraint " << std::endl;
                    leftPicksAndConstraints = getCommandLinePickAndConstraint(
                            optionalArguments, leftJointsAndConstraintsOpen, constraintsDelimiter, closingDelim, jointIdDelimiter,
                            constraintsCloseDelimiter, constraintsBeginSequence);

                    rightPicksAndConstraints = getCommandLinePickAndConstraint(
                            optionalArguments, rightJointsAndConstraintsOpen, constraintsDelimiter, closingDelim, jointIdDelimiter,
                            constraintsCloseDelimiter, constraintsBeginSequence);

                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, leftPicksAndConstraints,
                                                                         rightPicksAndConstraints, outputName);
                    writeAcquisition(ikAcq, argv[2]);
                    outputDifferenceCSV(acq, ikAcq, outputFileName + "_delta.csv");
                    std::cout << "Test constraint complete" << std::endl;
                } else {
                    btk::Acquisition::Pointer ikAcq = writeIkConstrained(acq, leftPicksAndConstraints,
                                                                         rightPicksAndConstraints, outputName);
                    writeAcquisition(ikAcq, argv[2]);
                    std::cout << "Just constrained" << std::endl;
                }
            }
        } else if (optionalArguments.find(testRegexStr) != std::string::npos) {
            std::cout << "Test regex" << std::endl;
            std::cout << "unconstrained test" << std::endl;
            btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/forward_jumping_jacks.c3d");
            btk::Acquisition::Pointer ikAcq = writeIkUnconstrained(acq, rightPicksInt, leftPicksInt);
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
