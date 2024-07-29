// Order of the include statements matters - especially when utilising UDP
#include <KinovaInit.h>				// always comes before all the other includes
#include <myKINOVA_includes.h>		// baked in
#include <myKINOVA_UDP.h>			// baked in
#include <myKINOVA_LOGGING.h>		// baked in
#include <myPARAMS.h>
#include <myKINOVA.h>			// baked in

int main()
{
	//printf("Enter IP Address of the robot\n");
	//char myIP_input[20];
	//scanf("%[^\n]%*c", myIP_input);
	//printf("IP Address entered is : %s\n", myIP_input);

	std::string ROBOT_IP_in = "192.180.0.107";
	//(myIP_input);

	//Kinova with Robotiq 2f 85 Gripper
	const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_GRIPPER_2024.urdf";
	int CTRL_MODE = 5;
	int DURATION = 1000;
	myPARAMS params_struct = setPARAMS(robot_model, ROBOT_IP_in, CTRL_MODE, 27015, 27016, "127.0.0.1", "127.0.0.1", DURATION, TRUE);
	float myK = 20;
	float myB = 3;
	myKINOVA ROBOT(params_struct, myK, myB);

	std::thread t1(&myKINOVA::ROBOT_Gq, &ROBOT, TRUE);

	t1.join();

	return 0;
}