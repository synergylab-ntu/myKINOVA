
#include <string>

#include <BaseClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;

#include <thread>

class myKINOVA_CMD {
public:
	int CTRL_MODE = 0;
	// CMD variables
	float tau_cmd[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float ext_tau[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float tau_ctrl[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float des_q[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float des_gripperPOS = 0.0f;

	myKINOVA_CMD() { // important to keep this default constructor, else you cannot define it as a variable in another class.

	}

	myKINOVA_CMD(int CTRL_MODE_IN) {
		CTRL_MODE = CTRL_MODE_IN;
	}
};

class myKINOVA {

public:

	
	// Kinova API variables
	KINOVA* robot_tcp;
	KINOVA* robot_udp;
	bool return_status;
	std::shared_ptr<rl::mdl::Model> model;
	rl::mdl::Kinematic* kinematic;
	rl::mdl::Dynamic* dynamics;
	rl::math::Vector q, qd, qdd, tau, K, B, des_q, ext_tau_limit;
	size_t DOF;
	k_api::BaseCyclic::Feedback base_feedback;
	k_api::BaseCyclic::Command  base_command;
	k_api::GripperCyclic::MotorCommand* m_gripper_motor_command;
	std::future<k_api::BaseCyclic::Feedback> base_feedback_async;
	k_api::Base::ServoingModeInformation servoing_mode;
	k_api::Base::BaseClient* base;
	k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
	k_api::ActuatorConfig::ActuatorConfigClient* actuator_config;
	//bool gripper;
	k_api::ActuatorConfig::ControlModeInformation control_mode_message;
	int role;
	float myK = 10, myB = 6;
	

	//// data management variables
	int i;
	float gripper_position;

	int ACTUATOR_COUNT = 7;
	
	float tau_fbk[7], Gq[7];

	myKINOVA_CMD ROB_CMD;
	myROB_PARAMS ROB_PARAMS;
	myKINOVA_LOG ROB_LOG;
	myKINOVA_UDP ROB_UDP;

	myKINOVA_CMD set_ROB_CMD(int CTRL_MODE_IN) {
		myKINOVA_CMD my_ROB_CMD(CTRL_MODE_IN);
		return my_ROB_CMD;
	}

	myROB_PARAMS set_ROB_PARAMS(myPARAMS PARAMS_IN) {
		myROB_PARAMS my_ROB_PARAMS(PARAMS_IN);
		return my_ROB_PARAMS;
	}

	myKINOVA_LOG set_ROB_LOG(int DURATION_IN) {
		myKINOVA_LOG my_ROB_LOG(DURATION_IN);
		return my_ROB_LOG;
	}

	myKINOVA_UDP set_ROB_UDP() {
		myKINOVA_UDP my_ROB_UDP(ROB_CMD.CTRL_MODE, ROB_PARAMS.SEND_PORT, ROB_PARAMS.RECV_PORT, ROB_PARAMS.SEND_IP_ADDRESS, ROB_PARAMS.RECV_IP_ADDRESS);
		return my_ROB_UDP;
	}

	bool clearfaults(k_api::Base::BaseClient* base_in) {
		return_status = true;
		try
		{
			base_in->ClearFaults();
		}
		catch (...)
		{
			std::cout << "Unable to clear robot faults" << std::endl;
			return false;
		}
		auto servoing_mode = k_api::Base::ServoingModeInformation();
	}

	std::shared_ptr<rl::mdl::Model> get_model(std::string robot_model_in) {
		rl::mdl::UrdfFactory factory;
		std::cout << "im here" << std::endl;
		std::shared_ptr<rl::mdl::Model> mymodel(factory.create(robot_model_in));
		std::cout << "mode here" << mymodel.get() << std::endl;

		//rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());
		kinematic = dynamic_cast<rl::mdl::Kinematic*>(mymodel.get());
		dynamics = dynamic_cast<rl::mdl::Dynamic*>(mymodel.get());

		std::cout << "im here2" << std::endl;
		DOF = dynamics->getDof();
		std::cout << DOF << endl;

		return mymodel;
	}

	void setDEFAULTPARAMS() {
		q = rl::math::Vector(DOF);
		qd = rl::math::Vector(DOF);
		qdd = rl::math::Vector(DOF);
		tau = rl::math::Vector(DOF);
		B = rl::math::Vector(DOF);
		K = rl::math::Vector(DOF);
		ext_tau_limit = rl::math::Vector(DOF);

		std::cout << "im here2" << std::endl;
		std::cout << DOF << endl;

		ext_tau_limit << 3, 2, 2, 5, 4, 3, 6;
		q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		tau << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		//B << 5.5220, 5.7140, 5.2075, 5.4825, 4.7267, 4.7631, 7.6092;
		B << myB, myB, myB, myB, myB, myB, myB;
		
		K << myK, myK, myK, myK, myK, myK, myK;


		std::cout << "im here3" << std::endl;

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);

	}

	void setupROB() {
		robot_tcp = new KINOVA(ROB_PARAMS.ROBOT_IP, 10000);
		robot_tcp->Init_TCP();

		robot_udp = new KINOVA(ROB_PARAMS.ROBOT_IP, 10001);
		robot_udp->Init_UDP();

		std::cout << "Creating Client" << std::endl;
		base = get_base(robot_tcp);
		base_cyclic = get_basecyclic(robot_udp);
		actuator_config = get_actuatorconfig(robot_tcp);
		std::cout << "Client Created" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // especially for goHOME(base) was called, but still keep it even if commented
	}

	k_api::Base::BaseClient* get_base(KINOVA* protocol) {
		auto mybase = new k_api::Base::BaseClient(protocol->get_router());
		return mybase;
	}

	k_api::BaseCyclic::BaseCyclicClient* get_basecyclic(KINOVA* protocol) {
		auto my_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(protocol->get_router());
		return my_base_cyclic;
	}

	k_api::ActuatorConfig::ActuatorConfigClient* get_actuatorconfig(KINOVA* protocol) {
		auto my_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(protocol->get_router());
		return my_actuator_config;
	}

	void init_logging_vars() {
		ROB_LOG.timer_count = 0;
		ROB_LOG.data_count = 0;
		ROB_LOG.now = 0;
		ROB_LOG.last = 0;
		ROB_LOG.logging = 1;      //change to zero to manually start logging
		ROB_LOG.focus = 0;
	}

	void setLOWLEVEL() {
		std::cout << "Setting low level servoing" << std::endl;
		// Set the base in low-level servoing mode
		servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
		base->SetServoingMode(servoing_mode);
		base_feedback = base_cyclic->RefreshFeedback();
		std::cout << "Initialising to current position" << std::endl;
	}

	void getFBK() {
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
			qdd[i] = 0;
			tau[i] = base_feedback.actuators(i).torque();
			tau_fbk[i] = tau[i];

			
			// add tau also
		}

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);
		dynamics->inverseDynamics();
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			Gq[i] = dynamics->getTorque()[i];
		}

		ROB_UDP = setUDP(ROB_UDP);
	}

	void init_actuators() {
		// Initialize each actuator to their current position
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			// Save the current actuator position, to avoid a following error
			base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
			ROB_CMD.des_q[i] = q[i]; // initialising the des_q to the home configuration you start from - VERY IMPORTANT
			ROB_CMD.tau_ctrl[i] = 0;
		}

		// Initialise the gripper position, velocity and force
		if (ROB_PARAMS.gripper_val)
		{
			float gripper_initial_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
			base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
			m_gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
			m_gripper_motor_command->set_position(gripper_initial_position);
			m_gripper_motor_command->set_velocity(0.0);
			m_gripper_motor_command->set_force(100.0);
		}

		std::cout << "Sending first frame" << std::endl;
		// Send a first frame
		base_feedback_async = base_cyclic->Refresh_async(base_command);
	}

	myKINOVA_UDP init_UDP_VARS(myKINOVA_UDP ROBOT_UDP_in) {
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			ROBOT_UDP_in.num_TOSEND[i] = q[i];
			ROBOT_UDP_in.UDP_q[i] = q[i];
		}
		return ROBOT_UDP_in;
	}

	myKINOVA_UDP setUDP(myKINOVA_UDP ROBOT_UDP_in) {
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			ROBOT_UDP_in.num_TOSEND[i] = q[i];
		}
		return ROBOT_UDP_in;
	}

	void init_TORQUE_CONTROL() {
		std::cout << "Change Control Mode - Torque" << std::endl;
		std::cout << "Starting Gravity Compensation" << std::endl;

		// Set actuator in torque mode now that the command is equal to measure
		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			if (i >= ROB_LOG.focus)
			{
				actuator_config->SetControlMode(control_mode_message, i + 1);
			}
		}
	}

	void setCONTROL() {

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);
		dynamics->inverseDynamics();

		for (i = 0; i < ACTUATOR_COUNT; ++i)
		{

			if (i >= ROB_LOG.focus)
			{
				if (ROB_CMD.CTRL_MODE == 0) // standard IMPEDANCE CONTROL (0)
				{
					ROB_CMD.ext_tau[i] = (K[i] * calculate_delta_q(ROB_CMD.des_q[i], q[i], 'r'));
					
				}
				else if (ROB_CMD.CTRL_MODE == 1)
				{
					ROB_CMD.ext_tau[i] = (K[i] * calculate_delta_q(ROB_CMD.des_q[i], q[i], 'r')) + ROB_CMD.tau_ctrl[i];
				}
				else if (ROB_CMD.CTRL_MODE == 2)
				{
					ROB_CMD.ext_tau[i] = ROB_CMD.tau_ctrl[i];
				}
				else if (ROB_CMD.CTRL_MODE == 3) // gravity compensation
				{
					ROB_CMD.ext_tau[i] = 0;
				}
				else if (ROB_CMD.CTRL_MODE == 4) // Teleoperation (4)
				{
					if (role == 0) {
						ROB_CMD.ext_tau[i] = ROB_CMD.tau_ctrl[i]; // for the master robot
					}
					else if (role == 1) {
						ROB_CMD.ext_tau[i] = (K[i] * calculate_delta_q(ROB_CMD.des_q[i], q[i], 'r')); // for the slave robot
					}
				}
				else if (ROB_CMD.CTRL_MODE == 5)
				{
					ROB_CMD.ext_tau[i] = (K[i] * calculate_delta_q(ROB_CMD.des_q[i], q[i], 'r'));
				}

				if (ROB_CMD.CTRL_MODE == 4) {
					// no filter here
				}
				else {
					if (ROB_CMD.ext_tau[i] > ext_tau_limit[i])
					{
						ROB_CMD.ext_tau[i] = ext_tau_limit[i];
					}

					if (ROB_CMD.ext_tau[i] < -ext_tau_limit[i])
					{
						ROB_CMD.ext_tau[i] = -ext_tau_limit[i];
					}
				}

				if (ROB_CMD.CTRL_MODE == 4) {

					if (role == 0) {// yes damping for master
						ROB_CMD.tau_cmd[i] = dynamics->getTorque()[i] + (B[i] * qd[i]) + ROB_CMD.ext_tau[i];
					}
					else if (role == 1) {// no damping for slave
						ROB_CMD.tau_cmd[i] = dynamics->getTorque()[i] + ROB_CMD.ext_tau[i];
					}
				}
				else { // for all other control policies
					ROB_CMD.tau_cmd[i] = dynamics->getTorque()[i] + (B[i] * qd[i]) + ROB_CMD.ext_tau[i];
				}

				base_command.mutable_actuators(i)->set_torque_joint(ROB_CMD.tau_cmd[i]);
				base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
			}
		}

		misc_KB_inputs();
		gripper_KB_CMD();
		gripper_UDP_CMD();
	}

	void gripper_UDP_CMD() {
		if (ROB_CMD.CTRL_MODE == 5) {
			if (ROB_PARAMS.gripper_val)
			{
				gripper_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
				std::cout << "Gripper command should be :" << ROB_CMD.des_gripperPOS << std::endl;
				m_gripper_motor_command->set_position(ROB_CMD.des_gripperPOS);
				m_gripper_motor_command->set_velocity(40.0);
				m_gripper_motor_command->set_force(100.0);
			}
		}
	}

	void gripper_KB_CMD() {
		if (ROB_CMD.CTRL_MODE == 5)
		{ }
		else {
			if (ROB_PARAMS.gripper_val)
			{
				gripper_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
				if (GetKeyState('W') & 0x8000)
				{
					if (gripper_position + 10.0 > 100.0)
						m_gripper_motor_command->set_position(100.0);
					else
						m_gripper_motor_command->set_position(gripper_position + 10.0);
					m_gripper_motor_command->set_velocity(40.0);
					m_gripper_motor_command->set_force(100.0);
				}
				if (GetKeyState('S') & 0x8000)
				{
					if (gripper_position - 10.0 < 0.0)
						m_gripper_motor_command->set_position(0.0);
					else
						m_gripper_motor_command->set_position(gripper_position - 10.0);
					m_gripper_motor_command->set_velocity(40.0);
					m_gripper_motor_command->set_force(1.0);
				}
			}
		}
	}

	void setcommand_FRAMEID() {
		//setting commmand frame id
		base_command.set_frame_id(base_command.frame_id() + 1);
		if (base_command.frame_id() > 65535)
			base_command.set_frame_id(0);

		for (int idx = 0; idx < ACTUATOR_COUNT; ++idx)
		{
			base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
		}

		try
		{
			base_feedback_async = base_cyclic->Refresh_async(base_command, 0);
		}
		catch (k_api::KDetailedException& ex)
		{
			std::cout << "Kortex exception: " << ex.what() << std::endl;

			std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
		}
		catch (std::runtime_error& ex2)
		{
			std::cout << "runtime error: " << ex2.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "Unknown error." << std::endl;
		}
	}

	void reset_SERVOING_MODE() {

		ROB_LOG.data_count_final = ROB_LOG.data_count;
		std::cout << "Ending Gravity Compensation" << std::endl;
		std::cout << "Change Control Mode - Position" << std::endl;
		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			actuator_config->SetControlMode(control_mode_message, i + 1);
		}

		std::cout << "Setting single level servoing" << std::endl;
		//Set the servoing mode back to Single Level
		servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
		base->SetServoingMode(servoing_mode);
	}

	void quickSETUP() {
		auto error_callback = [](k_api::KError err) { std::cout << "_________ callback error _________" << err.toString(); };
		setupROB();

		// Clearing faults
		clearfaults(base);

		// get URDF model
		model = get_model(ROB_PARAMS.robot_model);

		// set DEFAULT PARAMS
		setDEFAULTPARAMS();
	}

	void initialize_torque_control() {
		setLOWLEVEL();

		// Initialize each actuator to their current position
		init_actuators();

		init_TORQUE_CONTROL();
	}

	void misc_KB_inputs() {
		if (GetKeyState('L') & 0x8000)
		{
			ROB_LOG.logging = 1;
		}
	}

	float calculate_delta_q(float q_m, float q_s, char unit) {
		float del_q = q_m - q_s;
		if (unit == 'd')
		{
			if (abs(del_q) <= 180.0)
			{
				return del_q;
			}
			else if (q_m < 180.0)
			{
				return del_q + 360.0;
			}
			else
			{
				return del_q - 360.0;
			}
		}
		else if (unit == 'r')
		{
			if (abs(del_q) <= M_PI)
			{
				return del_q;
			}
			else if (q_m < M_PI)
			{
				return del_q + (2 * M_PI);
			}
			else
			{
				return del_q - (2 * M_PI);
			}
		}
	}

	bool goHOME(k_api::Base::BaseClient* base_in)
	{
		// Make sure the arm is in Single Level Servoing before executing an Action
		auto servoingMode = k_api::Base::ServoingModeInformation();
		servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
		base_in->SetServoingMode(servoingMode);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		// Move arm to ready position
		std::cout << "Moving the arm to a safe position" << std::endl;
		auto action_type = k_api::Base::RequestedActionType();
		action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
		auto action_list = base_in->ReadAllActions(action_type);
		auto action_handle = k_api::Base::ActionHandle();
		action_handle.set_identifier(0);
		for (k_api::Base::Action action : action_list.action_list())
		{
			if (action.name() == "Home")
			{
				action_handle = action.handle();
			}
		}

		if (action_handle.identifier() == 0)
		{
			std::cout << "Can't reach safe position, exiting" << std::endl;
			return false;
		}
		else
		{
			// Connect to notification action topic
			std::promise<k_api::Base::ActionEvent> promise;
			auto future = promise.get_future();
			const auto notification_handle = base_in->OnNotificationActionTopic(
				create_action_event_listener_by_promise(promise),
				k_api::Common::NotificationOptions{}
			);

			base_in->ExecuteActionFromReference(action_handle);

			// Wait for action to finish
			const auto status = future.wait_for(TIMEOUT_DURATION);
			base_in->Unsubscribe(notification_handle);

			if (status != std::future_status::ready)
			{
				std::cout << "Timeout on action notification wait" << std::endl;
				return false;
			}

			return true;

		}
	}

	std::function<void(k_api::Base::ActionNotification)> create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
	{
		return [&finish_promise](k_api::Base::ActionNotification notification)
		{
			const auto action_event = notification.action_event();
			switch (action_event)
			{
			case k_api::Base::ActionEvent::ACTION_END:
			case k_api::Base::ActionEvent::ACTION_ABORT:
				finish_promise.set_value(action_event);
				break;
			default:
				break;
			}
		};
	}

	void init_LOG() {
		init_logging_vars();
		ROB_LOG = set_ROB_LOG(ROB_PARAMS.DURATION);
	}

	void init_UDP() {
		ROB_UDP = set_ROB_UDP();
		ROB_UDP.setup_UDP();
		ROB_UDP = init_UDP_VARS(ROB_UDP);
	}

	void LOG_IT() {
		if (ROB_LOG.timer_count % 1 == 0 && ROB_LOG.data_count < ROB_PARAMS.DURATION * 1000 && ROB_LOG.logging == 1)
		{
			ROB_LOG.write2LOG(ROB_LOG.data_count, base_feedback, ROB_CMD.ext_tau, ROB_UDP.UDP_q, ROB_UDP.UDP_tau, ROB_CMD.tau_cmd);
			ROB_UDP.UDP_send_recv_v4(ROB_UDP.num_TOSEND);
			++ROB_LOG.data_count;
		}
	}

	void sendCMD() {
		setCONTROL();
		setcommand_FRAMEID();
		++ROB_LOG.timer_count;
		ROB_LOG.last = ROB_LOG.GetTickUs();
	}

	void myCLEANUP() {
		ROB_UDP.cleanup();
		reset_SERVOING_MODE();
	}

	void myWRITE() {
		//Save logs
		myWRITE_KINOVA_LOG LOG_WRITER(ROB_PARAMS.DURATION, ROB_PARAMS.ROBOT_IP, ROB_LOG.data_count_final);
		LOG_WRITER.write2FILE(ROB_LOG, model);

		// Wait for a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}

	void setCMD(float cmd_input[7]) {
		if (ROB_PARAMS.CTRL_MODE < 3) { // only for MATLAB real time control of the robot using impedance control
			for (i = 0; i < ACTUATOR_COUNT; ++i)
			{
				ROB_CMD.des_q[i] = ROB_UDP.UDP_q[i];
				ROB_CMD.tau_ctrl[i] = ROB_UDP.UDP_tau[i];
			}
		}

		if (ROB_PARAMS.CTRL_MODE == 3) { // only for gravity compensation
			for (i = 0; i < ACTUATOR_COUNT; ++i)
			{
				ROB_CMD.des_q[i] = 0;
				ROB_CMD.tau_ctrl[i] = 0;
			}
		}

		if (ROB_PARAMS.CTRL_MODE == 5) { // real time control with gripper input from MATLAB v4
			for (i = 0; i < ACTUATOR_COUNT; ++i)
			{
				ROB_CMD.des_q[i] = ROB_UDP.UDP_q[i];
				ROB_CMD.tau_ctrl[i] = ROB_UDP.UDP_tau[i];
			}
			ROB_CMD.des_gripperPOS = ROB_UDP.UDP_gripper;
			std::cout << "ROB_UDP gripper value is : " << ROB_UDP.UDP_gripper << std::endl;
		}

		//if (ROB_PARAMS.CTRL_MODE == 4) { // only for teleoperation
		//	for (i = 0; i < ACTUATOR_COUNT; ++i)
		//	{
		//		if (role == 0) {
		//			ROB_CMD.tau_ctrl[i] = cmd_input[i]; // master expects a torque feedback from slave
		//		}
		//		else if (role == 1) {
		//			ROB_CMD.des_q[i] = cmd_input[i]; // slave expects a position command from master
		//			ROB_CMD.tau_ctrl[i] = 0;
		//		}
		//	}
		//}

	}

	bool ROBOT_Gq(bool gripper_in) {
		ROB_PARAMS.gripper_val = gripper_in;
		quickSETUP();
		init_LOG();

		try
		{// initialize low level torque control and UDP communication
			initialize_torque_control();
			init_UDP();
			//Realtime control loop. Press "Q" key to exit loop.
			while (!(GetKeyState('Q') & 0x8000))
			{
				ROB_LOG.now = ROB_LOG.GetTickUs();
				if (ROB_LOG.now - ROB_LOG.last >= 1000)
				{
					//-----------------------------------------------------------------------------//
					base_feedback = base_feedback_async.get();

					LOG_IT();
					
					getFBK();
					setCMD(ROB_CMD.des_q); // should discern between using MATLAB UDP input as command or teleoperation input as command
					
					sendCMD();
				}
			}
		}
		catch (k_api::KDetailedException& ex)
		{
			std::cout << "API error: " << ex.what() << std::endl;
			return_status = false;
		}
		catch (std::runtime_error& ex2)
		{
			std::cout << "Error: " << ex2.what() << std::endl;
			return_status = false;
		}

		myCLEANUP();
		myWRITE();
		
		return return_status;
	}

	myKINOVA() {// default constructor
		myK = 10;
	}

	myKINOVA(std::string robot_model_in, std::string ROBOT_IP_in, int CTRL_MODE_IN, u_short SEND_PORT_IN, u_short RECV_PORT_IN, const char* SEND_IP_ADDRESS_IN, const char* RECV_IP_ADDRESS_IN, int DURATION_IN, bool gripper_val) {
		ROB_CMD = set_ROB_CMD(CTRL_MODE_IN);		
		ROB_PARAMS = set_ROB_PARAMS(setPARAMS(robot_model_in, ROBOT_IP_in, CTRL_MODE_IN, SEND_PORT_IN, RECV_PORT_IN, SEND_IP_ADDRESS_IN, RECV_IP_ADDRESS_IN, DURATION_IN, gripper_val));
		myK = 10;
	}

	myKINOVA(std::string robot_model_in, std::string ROBOT_IP_in, int CTRL_MODE_IN, u_short SEND_PORT_IN, u_short RECV_PORT_IN, const char* SEND_IP_ADDRESS_IN, const char* RECV_IP_ADDRESS_IN, int DURATION_IN, bool gripper_val, int role_in) {
		ROB_CMD = set_ROB_CMD(CTRL_MODE_IN);
		ROB_PARAMS = set_ROB_PARAMS(setPARAMS(robot_model_in, ROBOT_IP_in, CTRL_MODE_IN, SEND_PORT_IN, RECV_PORT_IN, SEND_IP_ADDRESS_IN, RECV_IP_ADDRESS_IN, DURATION_IN, gripper_val));
		role = role_in;
		myK = 10;
	}

	myKINOVA(myPARAMS PARAMS_IN) {
		ROB_CMD = set_ROB_CMD(PARAMS_IN.CTRL_MODE);
		ROB_PARAMS = set_ROB_PARAMS(PARAMS_IN);
		myK = 10;
	}

	myKINOVA(myPARAMS PARAMS_IN, float k_in) {
		ROB_CMD = set_ROB_CMD(PARAMS_IN.CTRL_MODE);
		ROB_PARAMS = set_ROB_PARAMS(PARAMS_IN);
		myK = k_in;
	}

	myKINOVA(myPARAMS PARAMS_IN, float k_in, float b_in) {
		ROB_CMD = set_ROB_CMD(PARAMS_IN.CTRL_MODE);
		ROB_PARAMS = set_ROB_PARAMS(PARAMS_IN);
		myK = k_in;
		myB = b_in;
	}

	myKINOVA(myPARAMS PARAMS_IN, int role_in) {
		ROB_CMD = set_ROB_CMD(PARAMS_IN.CTRL_MODE);
		ROB_PARAMS = set_ROB_PARAMS(PARAMS_IN);
		role = role_in;
		myK = 10;
	}

	myKINOVA(myPARAMS PARAMS_IN, int role_in, float k_in) {
		ROB_CMD = set_ROB_CMD(PARAMS_IN.CTRL_MODE);
		ROB_PARAMS = set_ROB_PARAMS(PARAMS_IN);
		role = role_in;
		myK = k_in;
	}
};