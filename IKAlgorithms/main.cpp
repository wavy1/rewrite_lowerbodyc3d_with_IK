#include <btkAcquisitionFileReader.h>
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

    btk::Acquisition::PointIterator itP = acq->FindPoint("Skeleton_001:WaistLFront");
    btk::PointCollection::Pointer marker_pts = acq->GetPoints();

    if(marker_pts->GetItemNumber() != 0) {        
        for(size_t f = 0;
            f < marker_pts->GetFrontItem()->GetFrameNumber();
            ++f) {
            for(btk::Acquisition::PointIterator it = marker_pts->Begin();
                it != marker_pts->End();
                ++it) {
                btk::Point::Pointer pt = *it;
                if(pt->GetLabel() == labelStr){
                    std::cout << "["<< f << "] "<< pt->GetLabel() << " " << "x: "<< pt->GetValues().coeff(f, 0) << " y: " << pt->GetValues().coeff(f, 1) << " z: " << pt->GetValues().coeff(f, 2) << std::endl;
                }
            }
        }
    }      
}

int main()
{
	btk::Acquisition::Pointer acq = readAcquisition("/vagrant/data/test_data.c3d");
    printPoints(acq);
    printPointsWithLabel(acq, "Skeleton_001:WaistLFront");
    

    struct ik_solver_t* solver = ik.solver.create(IK_FABRIK);

    /* Create a simple 3-bone structure */
    struct ik_node_t* root = solver->node->create(0);
    struct ik_node_t* child1 = solver->node->create_child(root, 1);
    struct ik_node_t* child2 = solver->node->create_child(child1, 2);
    struct ik_node_t* child3 = solver->node->create_child(child2, 3);

    /* Set node positions in local space so they form a straight line in the Y direction*/
    child1->position = ik.vec3.vec3(0, 10, 0);
    child2->position = ik.vec3.vec3(0, 10, 0);
    child3->position = ik.vec3.vec3(0, 10, 0);

    /* Attach an effector at the end */
    struct ik_effector_t* eff = solver->effector->create();
    solver->effector->attach(eff, child3);

    /* set the target position of the effector to be somewhere within range */
    eff->target_position = ik.vec3.vec3(2, -3, 5);

    /* We want to calculate rotations as well as positions */
    solver->flags |= IK_ENABLE_TARGET_ROTATIONS;

    /* Assign our tree to the solver, rebuild data and calculate solution */
    ik.solver.set_tree(solver, root);
    ik.solver.rebuild(solver);
    ik.solver.solve(solver);
	return 0;
};
