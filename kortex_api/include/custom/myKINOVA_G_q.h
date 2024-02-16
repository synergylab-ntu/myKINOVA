
#include <thread>

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

class myKINOVA {

public:
	// 
	int ACTUATOR_COUNT = 7;
	int DURATION;
	int CTRL_MODE = 0;

	// MATLAB communication parameters
	u_short SEND_PORT;
	u_short RECV_PORT;
	const char* SEND_IP_ADDRESS;
	const char* RECV_IP_ADDRESS;

	// Kinova API variables
	KINOVA* slave_tcp;
	KINOVA* slave_udp;
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
	bool gripper;
	k_api::ActuatorConfig::ControlModeInformation control_mode_message;

	// data management variables
	int i;
	int timer_count = 0;
	int data_count = 0;
	int64_t now = 0;
	int64_t last = 0;
	float gripper_position;
	int logging = 1;      //change to zero to manually start logging
	int focus = 0;

	// CMD variables
	float tau_cmd[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float ext_tau[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float tau_ctrl[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	int data_count_final;

	std::string ROBOT_IP;
	std::string robot_model;

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
		des_q = rl::math::Vector(DOF);
		ext_tau_limit = rl::math::Vector(DOF);

		std::cout << "im here2" << std::endl;
		std::cout << DOF << endl;

		des_q << 0, 0, 0, 0, 0, 0, 0;
		ext_tau_limit << 3, 2, 2, 5, 4, 3, 6;
		q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		tau << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		B << 5.5220, 5.7140, 5.2075, 5.4825, 4.7267, 4.7631, 7.6092;
		K << 10, 10, 10, 10, 10, 10, 10;


		std::cout << "im here3" << std::endl;

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);

	}

	void setupROB() {
		slave_tcp = new KINOVA(ROBOT_IP, 10000);
		slave_tcp->Init_TCP();

		slave_udp = new KINOVA(ROBOT_IP, 10001);
		slave_udp->Init_UDP();

		std::cout << "Creating Client" << std::endl;
		base = get_base(slave_tcp);
		base_cyclic = get_basecyclic(slave_udp);
		actuator_config = get_actuatorconfig(slave_tcp);
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
		timer_count = 0;
		data_count = 0;
		now = 0;
		last = 0;
		logging = 1;      //change to zero to manually start logging
		focus = 0;
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
			// add tau also
		}
	}

	void init_actuators() {
		// Initialize each actuator to their current position
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			// Save the current actuator position, to avoid a following error
			base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
			des_q[i] = q[i]; // initialising the des_q to the home configuration you start from - VERY IMPORTANT
			tau_ctrl[i] = 0;
		}

		// Initialise the gripper position, velocity and force
		if (gripper)
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

	myKINOVA_UDP init_UDP_VARS(myKINOVA_UDP ROBOT_UPD_in) {
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			ROBOT_UPD_in.num_TOSEND[i] = q[i];
			ROBOT_UPD_in.UDP_q[i] = q[i];
		}
		return ROBOT_UPD_in;
	}

	myKINOVA_UDP getsetUDP(myKINOVA_UDP ROBOT_UPD_in) {
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			ROBOT_UPD_in.num_TOSEND[i] = q[i];
			des_q[i] = ROBOT_UPD_in.UDP_q[i];
			tau_ctrl[i] = ROBOT_UPD_in.UDP_tau[i];
		}
		return ROBOT_UPD_in;
	}

	void init_TORQUE_CONTROL() {
		std::cout << "Change Control Mode - Torque" << std::endl;
		std::cout << "Starting Gravity Compensation" << std::endl;

		// Set actuator in torque mode now that the command is equal to measure
		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			if (i >= focus)
			{
				actuator_config->SetControlMode(control_mode_message, i + 1);
			}
		}
	}

	void setCMD() {

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);
		dynamics->inverseDynamics();

		for (i = 0; i < ACTUATOR_COUNT; ++i)
		{
			if (i >= focus)
			{

				if (CTRL_MODE == 0)
				{
					ext_tau[i] = (K[i] * calculate_delta_q(des_q[i], q[i], 'r'));
				}
				else if (CTRL_MODE == 1)
				{
					ext_tau[i] = (K[i] * calculate_delta_q(des_q[i], q[i], 'r')) + tau_ctrl[i];
				}
				else if (CTRL_MODE == 2)
				{
					ext_tau[i] = tau_ctrl[i];
				}

				if (ext_tau[i] > ext_tau_limit[i])
				{
					ext_tau[i] = ext_tau_limit[i];
				}

				if (ext_tau[i] < -ext_tau_limit[i])
				{
					ext_tau[i] = -ext_tau_limit[i];
				}

				tau_cmd[i] = dynamics->getTorque()[i] + (B[i] * qd[i]) + ext_tau[i];
				base_command.mutable_actuators(i)->set_torque_joint(tau_cmd[i]);
				base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
			}
		}
	}

	void gripper_KB_CMD() {
		if (gripper)
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

		data_count_final = data_count;
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
		setupROB();

		// Clearing faults
		clearfaults(base);

		// get URDF model
		model = get_model(robot_model);

		// set DEFAULT PARAMS
		setDEFAULTPARAMS();

		init_logging_vars();
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
			logging = 1;
		}
	}

	bool ROBOT_Gq(bool gripper_in) {
		gripper = gripper_in;
		auto error_callback = [](k_api::KError err) { std::cout << "_________ callback error _________" << err.toString(); };

		quickSETUP();

		myKINOVA_LOG ROBOT_LOG(DURATION);

		try
		{
			initialize_torque_control();

			// initialising control and communication variables
			myKINOVA_UDP ROBOT_UDP(CTRL_MODE, SEND_PORT, RECV_PORT, SEND_IP_ADDRESS, RECV_IP_ADDRESS);
			ROBOT_UDP.setup_UDP();

			ROBOT_UDP = init_UDP_VARS(ROBOT_UDP);

			//Realtime control loop. Press "Q" key to exit loop.
			while (!(GetKeyState('Q') & 0x8000))
			{
				now = ROBOT_LOG.GetTickUs();
				if (now - last >= 1000)
				{
					//-----------------------------------------------------------------------------//
					base_feedback = base_feedback_async.get();

					if (timer_count % 1 == 0 && data_count < DURATION * 1000 && logging == 1)
					{
						ROBOT_LOG.write2LOG(data_count, base_feedback, ext_tau, ROBOT_UDP.UDP_q, ROBOT_UDP.UDP_tau, tau_cmd);
						ROBOT_UDP.UDP_send_recv_v4(ROBOT_UDP.num_TOSEND);
						++data_count;
					}

					getFBK();
					ROBOT_UDP = getsetUDP(ROBOT_UDP);

					setCMD();

					misc_KB_inputs();

					gripper_KB_CMD();

					setcommand_FRAMEID();

					++timer_count;
					last = ROBOT_LOG.GetTickUs();
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

		reset_SERVOING_MODE();

		//Save logs
		myWRITE_KINOVA_LOG LOG_WRITER(DURATION, ROBOT_IP, data_count_final);
		LOG_WRITER.write2FILE(ROBOT_LOG, model);

		// Wait for a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		return return_status;
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

	myKINOVA(std::string robot_model_in, std::string ROBOT_IP_in, int CTRL_MODE_IN, u_short SEND_PORT_IN, u_short RECV_PORT_IN, const char* SEND_IP_ADDRESS_IN, const char* RECV_IP_ADDRESS_IN, int DURATION_IN, bool gripper_val) {
		robot_model = robot_model_in;
		ROBOT_IP = ROBOT_IP_in;
		DURATION = DURATION_IN;
		//ROBOT_Gq(gripper_val, DURATION_IN);
		SEND_IP_ADDRESS = SEND_IP_ADDRESS_IN;
		RECV_IP_ADDRESS = RECV_IP_ADDRESS_IN;
		SEND_PORT = SEND_PORT_IN;
		RECV_PORT = RECV_PORT_IN;
		CTRL_MODE = CTRL_MODE_IN;
	}

	myKINOVA(myPARAMS PARAMS_IN) {
		robot_model = PARAMS_IN.robot_model;
		ROBOT_IP = PARAMS_IN.ROBOT_IP;
		DURATION = PARAMS_IN.DURATION;
		//ROBOT_Gq(PARAMS_IN.gripper_val, PARAMS_IN.DURATION);
		SEND_IP_ADDRESS = PARAMS_IN.SEND_IP_ADDRESS;
		RECV_IP_ADDRESS = PARAMS_IN.RECV_IP_ADDRESS;
		SEND_PORT = PARAMS_IN.SEND_PORT;
		RECV_PORT = PARAMS_IN.RECV_PORT;
		CTRL_MODE = PARAMS_IN.CTRL_MODE;
	}
};