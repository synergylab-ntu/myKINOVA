
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
	int ACTUATOR_COUNT = 7;
	int DURATION;
	int CTRL_MODE = 0;

	u_short SEND_PORT;
	u_short RECV_PORT;

	const char* SEND_IP_ADDRESS;
	const char* RECV_IP_ADDRESS;

	KINOVA* slave_tcp;
	KINOVA* slave_udp;
	std::shared_ptr<rl::mdl::Model> model;
	float tau_cmd[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float ext_tau[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	float tau_ctrl[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	std::string ROBOT_IP;
	std::string robot_model;

	/*void getPOS() {
		for (int i = 0; i < ACTUATOR_COUNT; i++)
		{
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
		}
	}*/

	bool ROBOT_Gq(bool gripper, int DURATION_IN) {
		DURATION = DURATION_IN;

		auto error_callback = [](k_api::KError err) { std::cout << "_________ callback error _________" << err.toString(); };
		// ### TO DO ###: Change to correct IP
		slave_tcp = new KINOVA(ROBOT_IP, 10000);
		slave_tcp->Init_TCP();

		slave_udp = new KINOVA(ROBOT_IP, 10001);
		slave_udp->Init_UDP();

		std::cout << "Creating Client" << std::endl;

		auto base = new k_api::Base::BaseClient(slave_tcp->get_router());
		auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(slave_udp->get_router());
		auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(slave_tcp->get_router());
		std::cout << "Client Created" << std::endl;

		//goHOME(base);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // especially for goHOME(base) was called, but still keep it even if commented

		std::cout << "Initializing gravity compensation" << std::endl;

		bool return_status = true;

		// Clearing faults
		try
		{
			base->ClearFaults();
		}
		catch (...)
		{
			std::cout << "Unable to clear robot faults" << std::endl;
			return false;
		}

		rl::mdl::UrdfFactory factory;
		std::cout << "im here" << std::endl;

		std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model));
		std::cout << "mode here" << model.get() << std::endl;


		rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());
		rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());

		rl::math::Vector q(dynamics->getDof());
		rl::math::Vector qd(dynamics->getDof());
		rl::math::Vector qdd(dynamics->getDof());
		rl::math::Vector B(dynamics->getDof());
		rl::math::Vector K(dynamics->getDof());
		rl::math::Vector des_q(dynamics->getDof());
		rl::math::Vector ext_tau_limit(dynamics->getDof());
		std::cout << "im here2" << std::endl;
		std::cout << dynamics->getDof() << endl;

		des_q << 0, 0, 0, 0, 0, 0, 0;
		ext_tau_limit << 3, 2, 2, 5, 4, 3, 6;
		q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		B << 5.5220, 5.7140, 5.2075, 5.4825, 4.7267, 4.7631, 7.6092;
		K << 10, 10, 10, 10, 10, 10, 10;

		std::cout << "im here3" << std::endl;

		dynamics->setPosition(q);
		dynamics->setVelocity(qd);
		dynamics->setAcceleration(qdd);

		k_api::BaseCyclic::Feedback base_feedback;
		k_api::BaseCyclic::Command  base_command;
		k_api::GripperCyclic::MotorCommand* m_gripper_motor_command;

		std::future<k_api::BaseCyclic::Feedback> base_feedback_async;

		auto servoing_mode = k_api::Base::ServoingModeInformation();


		int i = 0;
		int timer_count = 0;
		int data_count = 0;
		int64_t now = 0;
		int64_t last = 0;
		float actual_position;
		int logging = 1;      //change to zero to manually start logging
		int focus = 0;


		myKINOVA_LOG ROBOT_LOG(DURATION);

		try
		{
			std::cout << "Setting low level servoing" << std::endl;
			// Set the base in low-level servoing mode
			servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
			base->SetServoingMode(servoing_mode);
			base_feedback = base_cyclic->RefreshFeedback();
			std::cout << "Initialising to current position" << std::endl;
			// Initialize each actuator to their current position
			for (i = 0; i < ACTUATOR_COUNT; i++)
			{
				// Save the current actuator position, to avoid a following error
				base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
				q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
				qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
				des_q[i] = q[i]; // initialising the des_q to the home configuration you start from - VERY IMPORTANT
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

			myKINOVA_UDP ROBOT_UDP(SEND_PORT, RECV_PORT, SEND_IP_ADDRESS, RECV_IP_ADDRESS);
			ROBOT_UDP.setup_UDP();

			for (i = 0; i < ACTUATOR_COUNT; i++)
			{
				q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
				ROBOT_UDP.num_TOSEND[i] = q[i];
				ROBOT_UDP.UDP_q[i] = q[i];
			}


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

					for (i = 0; i < ACTUATOR_COUNT; i++)
					{
						q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
						qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
						qdd[i] = 0.0;

						ROBOT_UDP.num_TOSEND[i] = q[i];
						des_q[i] = ROBOT_UDP.UDP_q[i];
						tau_ctrl[i] = ROBOT_UDP.UDP_tau[i];
					}

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

					// set rotation flag to 1 ---------------------------------------------
					if (GetKeyState('L') & 0x8000)
					{
						logging = 1;
					}

					if (gripper)
					{
						actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
						if (GetKeyState('W') & 0x8000)
						{
							if (actual_position + 10.0 > 100.0)
								m_gripper_motor_command->set_position(100.0);
							else
								m_gripper_motor_command->set_position(actual_position + 10.0);
							m_gripper_motor_command->set_velocity(40.0);
							m_gripper_motor_command->set_force(100.0);
						}
						if (GetKeyState('S') & 0x8000)
						{
							if (actual_position - 10.0 < 0.0)
								m_gripper_motor_command->set_position(0.0);
							else
								m_gripper_motor_command->set_position(actual_position - 10.0);
							m_gripper_motor_command->set_velocity(40.0);
							m_gripper_motor_command->set_force(1.0);
						}
					}
					//-----------------------------------------------------------------------------//

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

		//Save logs

		myWRITE_KINOVA_LOG LOG_WRITER(DURATION, ROBOT_IP);
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
		ROBOT_Gq(gripper_val, DURATION_IN);
		SEND_IP_ADDRESS = SEND_IP_ADDRESS_IN;
		RECV_IP_ADDRESS = RECV_IP_ADDRESS_IN;
		SEND_PORT = SEND_PORT_IN;
		RECV_PORT = RECV_PORT_IN;
		CTRL_MODE = CTRL_MODE_IN;
	}

	myKINOVA(myPARAMS PARAMS_IN) {
		robot_model = PARAMS_IN.robot_model;
		ROBOT_IP = PARAMS_IN.ROBOT_IP;
		ROBOT_Gq(PARAMS_IN.gripper_val, PARAMS_IN.DURATION);
		SEND_IP_ADDRESS = PARAMS_IN.SEND_IP_ADDRESS;
		RECV_IP_ADDRESS = PARAMS_IN.RECV_IP_ADDRESS;
		SEND_PORT = PARAMS_IN.SEND_PORT;
		RECV_PORT = PARAMS_IN.RECV_PORT;
		CTRL_MODE = PARAMS_IN.CTRL_MODE;
	}
};