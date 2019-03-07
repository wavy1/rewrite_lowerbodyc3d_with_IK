#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <iostream>
#include <typeinfo>

btk::Acquisition::Pointer readAcquisition(const std::string& filename)
{
	btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
	reader->SetFilename(filename);
	reader->Update();
	return reader->GetOutput();
};

void writeAcquisition(btk::Acquisition::Pointer acq, const std::string& filename)
{
  btk::AcquisitionFileWriter::Pointer writer = btk::AcquisitionFileWriter::New();
  writer->SetInput(acq);
  writer->SetFilename(filename);
  writer->Update();
};

btk::Acquisition::Pointer readC3d(const std::string& filename)
{
	btk::AcquisitionFileReader::Pointer reader = btk::AcquisitionFileReader::New();
	btk::C3DFileIO::Pointer io = btk::C3DFileIO::New();
	reader->SetAcquisitionIO(io);
	reader->SetFilename(filename);
	reader->Update();
}

std::string GetPointTypeAsString(btk::Point::Type t){
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

void printPoints(btk::Acquisition::Pointer acq){
	
    btk::PointCollection::Pointer marker_pts = acq->GetPoints();

    if(marker_pts->GetItemNumber() != 0) {        
        for(size_t f = 0;
            f < marker_pts->GetFrontItem()->GetFrameNumber();
            ++f) {
            for(btk::Acquisition::PointIterator it = marker_pts->Begin();
                it != marker_pts->End();
                ++it) {
                btk::Point::Pointer pt = *it;
                std::cout << "["<< f << "] "<< pt->GetLabel() << " " << "x: "<< pt->GetValues().coeff(f, 0) << " y: " << pt->GetValues().coeff(f, 1) << " z: " << pt->GetValues().coeff(f, 2) << std::endl;
            }
            std::cout << std::endl;
        }
    }      
}

void printPointsWithLabel(btk::Acquisition::Pointer acq, std::string labelStr){
    btk::Point::Pointer itP = acq->GetPoint(labelStr);

    if(acq->GetPointFrameNumber() != 0) {        
        for(size_t f = 0;
            f < acq->GetPointFrameNumber();
            ++f) {
                std::cout << "["<< f << "] "<< labelStr << " " << "x: "<< itP->GetValues().coeff(f, 0) << " y: " << itP->GetValues().coeff(f, 1) << " z: " << itP->GetValues().coeff(f, 2) << std::endl;
        }
    }      
}

std::vector<double> pointAt(btk::Point::Pointer itP, int frameNumber){
	std::vector<double> point;
	point.push_back(itP->GetValues().coeff(frameNumber, 0));
	point.push_back(itP->GetValues().coeff(frameNumber, 1));
	point.push_back(itP->GetValues().coeff(frameNumber, 2));
	return point;
}

btk::Acquisition::Pointer writeIk(btk::Acquisition::Pointer acq){
	btk::Acquisition::Pointer printAcquisition = acq->Clone();
	btk::Point::Pointer waistPoint = btk::Point::New("Skeleton_001:WaistLFront2", acq->GetPointFrameNumber());
	btk::Point::Pointer kneePoint = btk::Point::New("Skeleton_001:LKneeOut2", acq->GetPointFrameNumber());
	btk::Point::Pointer heelPoint = btk::Point::New("Skeleton_001:LHeel2", acq->GetPointFrameNumber());
	btk::Point::Pointer tipToePoint = btk::Point::New("Skeleton_001:LToeTip2", acq->GetPointFrameNumber());

	std::cout << acq->GetPointFrameNumber() << std::endl;
	std::vector<double> waist = pointAt(acq->GetPoint("Skeleton_001:WaistLFront"),0);
	std::vector<double> knee = pointAt(acq->GetPoint("Skeleton_001:LKneeOut"),0);
	std::vector<double> heel = pointAt(acq->GetPoint("Skeleton_001:LHeel"),0);
    std::vector<double> tipToePoint0 = pointAt(acq->GetPoint("Skeleton_001:LToeTip"),0);

    for(size_t index = 0; index < acq->GetPointFrameNumber() ; index++){
	    /* set the target position of the effector to be somewhere within range */
	    std::vector<double> toeTop = pointAt(acq->GetPoint("Skeleton_001:LToeTip"),index);
    printAcquisition->SetPoint(0, waistPoint);
    printAcquisition->SetPoint(4, kneePoint);
    printAcquisition->SetPoint(10, heelPoint);
    printAcquisition->SetPoint(11, tipToePoint);
   	return printAcquisition; 
}

int main(int argc, char** argv)
{
	btk::Acquisition::Pointer acq = readAcquisition(argv[1]);
   	btk::Acquisition::Pointer ikAcq = writeIk(acq);
   	writeAcquisition(ikAcq, argv[2]);
	return 0;
};
