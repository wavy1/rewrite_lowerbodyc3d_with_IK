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

std::vector<std::pair<int, btk::Point::Pointer> > pickFromAcq(btk::Acquisition::Pointer acq) {
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
        std::cout << "Added " << choice << " to the memory" << std::endl;
        choices.push_back(choice);
    }

    index = 0;

    for (btk::Acquisition::PointIterator acqIt = acq->BeginPoint(); acqIt != acq->EndPoint(); ++acqIt) {

        for (std::vector<int>::iterator idChoicesIt = choices.begin(); idChoicesIt != choices.end(); ++idChoicesIt) {
            if (*idChoicesIt == index) {
                std::cout << "Added " << *idChoicesIt << " to " << acqIt->get()->GetLabel() << " to the map"
                          << std::endl;
                idsAndPoints.push_back(std::make_pair(*idChoicesIt, acqIt->get()));
            }
        }

        index++;
    }

    return idsAndPoints;
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

btk::Acquisition::Pointer writeIkUnconstrainedWithOrigin(btk::Acquisition::Pointer acq) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks = pickFromAcq(acq);

    std::cout << "Choice right" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks = pickFromAcq(acq);

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
    leftOutputPoints = leftChain.prepareOutput(acq, leftPointPicks, leftOutputPoints);
    rightOutputPoints = rightChain.prepareOutput(acq, rightPointPicks, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve fabrikSolveRight(rightChain, tolerance, false);
    FabrikSolve fabrikSolveLeft(leftChain, tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(rightChain.pointAt(rightInputPoints.rbegin()->second, index));
        fabrikSolveRight.solve();

        fabrikSolveLeft.setTarget(leftChain.pointAt(leftInputPoints.rbegin()->second, index));
        fabrikSolveLeft.solve();

        size_t j = 0;

        while (j < fabrikSolveLeft.getJoints().size()) {
            std::cout << j << std::endl;
            leftOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveLeft.getJoints().at(j).coeff(0),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(1),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(2));
            rightOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveRight.getJoints().at(j).coeff(0),
                                                         fabrikSolveRight.getJoints().at(j).coeff(1),
                                                         fabrikSolveRight.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(leftOutputPoints.at(j).first, rightOutputPoints.at(j).second);
            printAcquisition->SetPoint(leftOutputPoints.at(j).first, leftOutputPoints.at(j).second);
            ++j;
        }

    }
    return printAcquisition;
}





btk::Acquisition::Pointer writeIkUnconstrained(btk::Acquisition::Pointer acq) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks = pickFromAcq(acq);

    std::cout << "Choice right" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks = pickFromAcq(acq);

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
    leftOutputPoints = leftChain.prepareOutput(acq, leftPointPicks, leftOutputPoints);
    rightOutputPoints = rightChain.prepareOutput(acq, rightPointPicks, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve fabrikSolveRight(rightChain, tolerance, false);
    FabrikSolve fabrikSolveLeft(leftChain, tolerance, false);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(rightChain.pointAt(rightInputPoints.rbegin()->second, index));
        fabrikSolveRight.solve();

        fabrikSolveLeft.setTarget(leftChain.pointAt(leftInputPoints.rbegin()->second, index));
        fabrikSolveLeft.solve();

        size_t j = 0;

        while (j < fabrikSolveLeft.getJoints().size()) {
            std::cout << j << std::endl;
            leftOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveLeft.getJoints().at(j).coeff(0),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(1),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(2));
            rightOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveRight.getJoints().at(j).coeff(0),
                                                         fabrikSolveRight.getJoints().at(j).coeff(1),
                                                         fabrikSolveRight.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(leftOutputPoints.at(j).first, rightOutputPoints.at(j).second);
            printAcquisition->SetPoint(leftOutputPoints.at(j).first, leftOutputPoints.at(j).second);
            ++j;
        }

    }
}


btk::Acquisition::Pointer writeIkConstrained(btk::Acquisition::Pointer acq) {
    AcquisitionChain rightChain;
    AcquisitionChain leftChain;
    float tolerance = 0.00001f;

    std::cout << "Choice left" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > leftPointPicks = pickFromAcq(acq);

    std::cout << "Choice right" << std::endl;
    std::vector<std::pair<int, btk::Point::Pointer> > rightPointPicks = pickFromAcq(acq);

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
    leftOutputPoints = leftChain.prepareOutput(acq, leftPointPicks, leftOutputPoints);
    rightOutputPoints = rightChain.prepareOutput(acq, rightPointPicks, rightOutputPoints);

    leftChain.calculateDistancesDebugless(leftChain.getPositionsPerFrame(0));
    rightChain.calculateDistancesDebugless(rightChain.getPositionsPerFrame(0));

    FabrikSolve fabrikSolveRight(rightChain, tolerance, true);
    FabrikSolve fabrikSolveLeft(leftChain, tolerance, true);
    fabrikSolveLeft.setAllConeConstraints(15,15,15,15);
    fabrikSolveRight.setAllConeConstraints(15,15,15,15);

    std::cout << "Frames: " << acq->GetPointFrameNumber() << std::endl;
    for (size_t index = 1; index + 1 < acq->GetPointFrameNumber(); index++) {
        std::cout << "Index: " << index << std::endl;

        fabrikSolveRight.setTarget(rightChain.pointAt(rightInputPoints.rbegin()->second, index));
        fabrikSolveRight.solve();

        fabrikSolveLeft.setTarget(leftChain.pointAt(leftInputPoints.rbegin()->second, index));
        fabrikSolveLeft.solve();

        size_t j = 0;

        while (j < fabrikSolveLeft.getJoints().size()) {
            std::cout << j << std::endl;
            leftOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveLeft.getJoints().at(j).coeff(0),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(1),
                                                        fabrikSolveLeft.getJoints().at(j).coeff(2));
            rightOutputPoints.at(j).second->SetDataSlice(index, fabrikSolveRight.getJoints().at(j).coeff(0),
                                                         fabrikSolveRight.getJoints().at(j).coeff(1),
                                                         fabrikSolveRight.getJoints().at(j).coeff(2));

            printAcquisition->SetPoint(leftOutputPoints.at(j).first, rightOutputPoints.at(j).second);
            printAcquisition->SetPoint(leftOutputPoints.at(j).first, leftOutputPoints.at(j).second);
            ++j;
        }

    }
}

int main(int argc, char **argv) {
    btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
    std::cout << argc << std::endl;


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
            btk::Acquisition::Pointer ikAcq = writeIkUnconstrainedWithOrigin(acq);
            writeAcquisition(ikAcq, argv[2]);
        }
    }

    return 0;
};
