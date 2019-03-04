#include <btkAcquisitionFileReader.h>
#include <btkAcquisitionFileWriter.h>
#include <btkAcquisition.h>
#include <btkC3DFileIO.h>
#include <iostream>
#include <typeinfo>
#include <ik.h>

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

void writeIk(btk::Acquisition::Pointer acq){
	btk::Acquisition::Pointer printAcquisition = acq->Clone();
	btk::Point::Pointer waistPoint = btk::Point::New("Skeleton_001:WaistLFront2", acq->GetPointFrameNumber());
	btk::Point::Pointer kneePoint = btk::Point::New("Skeleton_001:LKneeOut2", acq->GetPointFrameNumber());
	btk::Point::Pointer heelPoint = btk::Point::New("Skeleton_001:LHeel2", acq->GetPointFrameNumber());
	btk::Point::Pointer tipToePoint = btk::Point::New("Skeleton_001:LToeTip2", acq->GetPointFrameNumber());

	std::cout << acq->GetPointFrameNumber() << std::endl;
    struct ik_solver_t* solver = ik.solver.create(IK_FABRIK);

    /* Create a simple 3-bone structure */
    struct ik_node_t* root = solver->node->create(0);
    struct ik_node_t* child1 = solver->node->create_child(root, 1);
    struct ik_node_t* child2 = solver->node->create_child(child1, 2);
    struct ik_node_t* child3 = solver->node->create_child(child2, 3);

	std::vector<double> waist = pointAt(acq->GetPoint("Skeleton_001:WaistLFront"),0);
	std::vector<double> knee = pointAt(acq->GetPoint("Skeleton_001:LKneeOut"),0);
	std::vector<double> heel = pointAt(acq->GetPoint("Skeleton_001:LHeel"),0);

    /* Set node positions in local space so they form a straight line in the Y direction*/
    child1->position = ik.vec3.vec3(waist.at(0), waist.at(1), waist.at(2));
    child2->position = ik.vec3.vec3(knee.at(0), knee.at(1), knee.at(2));
    child3->position = ik.vec3.vec3(heel.at(0), heel.at(1), heel.at(2));

    /* Attach an effector at the end */
    struct ik_effector_t* eff = solver->effector->create();
    solver->effector->attach(eff, child3);
    solver->flags |= IK_ENABLE_TARGET_ROTATIONS;
    std::vector<double> tipToePoint0 = pointAt(acq->GetPoint("Skeleton_001:LToeTip"),0);
	eff->target_position = ik.vec3.vec3(tipToePoint0.at(0), tipToePoint0.at(1), tipToePoint0.at(2));
	ik.solver.set_tree(solver, root);

    for(size_t index = 0; index < acq->GetPointFrameNumber() ; index++){
	    /* set the target position of the effector to be somewhere within range */
	    std::vector<double> toeTop = pointAt(acq->GetPoint("Skeleton_001:LToeTip"),index);
	    eff->target_position = ik.vec3.vec3(toeTop.at(0), toeTop.at(1), toeTop.at(2));
	    /* We want to calculate rotations as well as positions */
	 
	    /* Assign our tree to the solver, rebuild data and calculate solution */
	    ik.solver.rebuild(solver);
	    ik.solver.solve(solver);
	    
	    waistPoint->SetDataSlice(index, child1->position.x, child1->position.y, child1->position.z);
		kneePoint->SetDataSlice(index, child2->position.x, child2->position.y, child2->position.z);
		heelPoint->SetDataSlice(index, child3->position.x, child3->position.y, child3->position.z);
		tipToePoint->SetDataSlice(index, toeTop.at(0), toeTop.at(1), toeTop.at(2));


	    std::cout << " " << child1->position.x << " " << child1->position.y << " " << child1->position.z << std::endl;
	    std::cout << " " << child2->position.x << " " << child2->position.y << " " << child2->position.z << std::endl;
	    std::cout << " " << child3->position.x << " " << child3->position.y << " " << child3->position.z << std::endl << std::endl;
    }
    printAcquisition->SetPoint(0, waistPoint);
    printAcquisition->SetPoint(4, kneePoint);
    printAcquisition->SetPoint(10, heelPoint);
    printAcquisition->SetPoint(11, tipToePoint);
    writeAcquisition(printAcquisition, "/home/vagrant/test_data.c3d");
}

int main()
{
	btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/test_data.c3d");
   	writeIk(acq);

	return 0;
};
