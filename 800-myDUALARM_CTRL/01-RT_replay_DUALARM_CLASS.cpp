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

	std::string L_ROB_IP = "192.180.0.109";
	std::string R_ROB_IP = "192.180.0.107";
	//(myIP_input);

	//Kinova with Robotiq 2f 85 Gripper
	const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_URDF_V12_fixed_rev.urdf"; // URDF for lightweight handle at the end-effector - @Qian : to make a new accurate URDF
	int CTRL_MODE = 5;
	int DURATION = 1000;
	
	myPARAMS L_params_struct = setPARAMS(robot_model, L_ROB_IP, CTRL_MODE, 27015, 27016, "127.0.0.1", "127.0.0.1", DURATION, TRUE);
	myPARAMS R_params_struct = setPARAMS(robot_model, R_ROB_IP, CTRL_MODE, 27017, 27018, "127.0.0.1", "127.0.0.1", DURATION, TRUE);

	float myK = 40;
	float myB = 3;
	
	myKINOVA L_ROBOT(L_params_struct, myK, myB);
	myKINOVA R_ROBOT(R_params_struct, myK, myB);

	std::thread t1(&myKINOVA::ROBOT_Gq, &L_ROBOT, TRUE);
	std::thread t2(&myKINOVA::ROBOT_Gq, &R_ROBOT, TRUE);

	t1.join();
	t2.join();

	return 0;
}