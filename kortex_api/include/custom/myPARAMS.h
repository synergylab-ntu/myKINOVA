

struct myPARAMS {
	std::string robot_model;
	std::string ROBOT_IP;
	int DURATION;
	bool gripper_val;
	u_short SEND_PORT;
	u_short RECV_PORT;
	const char* SEND_IP_ADDRESS;
	const char* RECV_IP_ADDRESS;
	int CTRL_MODE;
};

myPARAMS setPARAMS(std::string robot_model_in, std::string ROBOT_IP_in, int CTRL_MODE_IN, u_short SEND_PORT_IN, u_short RECV_PORT_IN, const char* SEND_IP_ADDRESS_IN, const char* RECV_IP_ADDRESS_IN, int DURATION_IN, bool gripper_val_in)
{
	myPARAMS params_struct;
	params_struct.robot_model = robot_model_in;
	params_struct.ROBOT_IP = ROBOT_IP_in;
	params_struct.DURATION = DURATION_IN;
	params_struct.gripper_val = gripper_val_in;
	params_struct.SEND_IP_ADDRESS = SEND_IP_ADDRESS_IN;
	params_struct.RECV_IP_ADDRESS = RECV_IP_ADDRESS_IN;
	params_struct.SEND_PORT = SEND_PORT_IN;
	params_struct.RECV_PORT = RECV_PORT_IN;
	params_struct.CTRL_MODE = CTRL_MODE_IN;
	return params_struct;
}


class myROB_PARAMS {

public:
	std::string robot_model;
	std::string ROBOT_IP;
	int DURATION;
	bool gripper_val;
	u_short SEND_PORT;
	u_short RECV_PORT;
	const char* SEND_IP_ADDRESS;
	const char* RECV_IP_ADDRESS;
	int CTRL_MODE;


	myROB_PARAMS() {

	}
	myROB_PARAMS(myPARAMS PARAMS_IN) {
		robot_model = PARAMS_IN.robot_model;
		ROBOT_IP = PARAMS_IN.ROBOT_IP;
		DURATION = PARAMS_IN.DURATION;
		SEND_IP_ADDRESS = PARAMS_IN.SEND_IP_ADDRESS;
		RECV_IP_ADDRESS = PARAMS_IN.RECV_IP_ADDRESS;
		SEND_PORT = PARAMS_IN.SEND_PORT;
		RECV_PORT = PARAMS_IN.RECV_PORT;
	}
};