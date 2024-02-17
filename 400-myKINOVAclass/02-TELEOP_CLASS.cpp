// Order of the include statements matters - especially when utilising UDP
#include <KinovaInit.h>				// always comes before all the other includes
#include <myKINOVA_includes.h>		// baked in
#include <myKINOVA_UDP.h>			// baked in
#include <myKINOVA_LOGGING.h>		// baked in
#include <myPARAMS.h>
#include <myKINOVA.h>			// baked in
#include <myTELEOP.h>

int main()
{
	int CTRL_MODE = 4;
	int DURATION = 1000;

	std::string MASTER_IP = "192.180.0.107";
	std::string SLAVE_IP = "192.180.0.108";

	//Kinova with Robotiq 2f 85 Gripper
	const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_GRIPPER_2024.urdf";

	myPARAMS master_PARAMS = setPARAMS(robot_model, MASTER_IP, CTRL_MODE, 27015, 27016, "127.0.0.1", "127.0.0.1", DURATION, TRUE);
	myPARAMS slave_PARAMS = setPARAMS(robot_model, SLAVE_IP, CTRL_MODE, 27017, 27018, "127.0.0.1", "127.0.0.1", DURATION, TRUE);

	myTELEOP TELEOP(master_PARAMS,slave_PARAMS);

	std::cout << TELEOP.MASTER.ROB_PARAMS.ROBOT_IP << std::endl;

//	std::thread t1(&myKINOVA::ROBOT_Gq, &ROBOT, TRUE);

	//t1.join();

	return 0;
}

