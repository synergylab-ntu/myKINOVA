
class myTELEOP {

public:
	myKINOVA MASTER;
	myKINOVA SLAVE;
	myPARAMS master_PARAMS;
	myPARAMS slave_PARAMS;
	int i;
	float tau_cmd_master[7];

	myKINOVA getROB_OBJ(myPARAMS PARAMS_IN, int role_in) {
		myKINOVA myROB(PARAMS_IN, role_in);
		return myROB;
	}

	myKINOVA getROB_OBJ_K(myPARAMS PARAMS_IN, int role_in, float k_in) {
		myKINOVA myROB(PARAMS_IN, role_in, k_in);
		return myROB;
	}

	bool teleoperate(bool gripper_in) {
		bool gripper = gripper_in;
		
		initiate_robots();

		try
		{// initialize low level torque control and UDP communication
			INIT_LOWLEVELnUDP();
			
			//Realtime control loop. Press "Q" key to exit loop.
			while (!(GetKeyState('Q') & 0x8000))
			{
				MASTER.ROB_LOG.now = MASTER.ROB_LOG.GetTickUs();
				if (MASTER.ROB_LOG.now - MASTER.ROB_LOG.last >= 1000)
				{
					//-----------------------------------------------------------------------------//
					MASTER.base_feedback = MASTER.base_feedback_async.get();
					SLAVE.base_feedback = SLAVE.base_feedback_async.get();

					TELEOP_LOGIT();

					TELEOP_GETFBK();

					setTELEOP_CMD(); // take the getFBK results and put them in their respective ROB_CMD tumblers

					TELEOP_sendCMD();
				}
			}
		}
		catch (k_api::KDetailedException& ex)
		{
			std::cout << "API error: " << ex.what() << std::endl;
			MASTER.return_status = false;
			SLAVE.return_status = false;
		}
		catch (std::runtime_error& ex2)
		{
			std::cout << "Error: " << ex2.what() << std::endl;
			MASTER.return_status = false;
			SLAVE.return_status = false;
		}

		CLEANUP();

		return MASTER.return_status;
	}

	void setTELEOP_CMD() {
		for (i = 0; i < MASTER.ACTUATOR_COUNT; i++)
		{
			SLAVE.ROB_CMD.des_q[i] = MASTER.q[i];								// slave expects a position command from master
			MASTER.ROB_CMD.tau_ctrl[i] = SLAVE.tau_fbk[i] + SLAVE.Gq[i];		// master expects a torque feedback from slave
		}
		//SLAVE.setCMD(SLAVE.ROB_CMD.des_q);
		//MASTER.setCMD(MASTER.ROB_CMD.des_q);
	}
	void initiate_robots() {
		MASTER.quickSETUP();
		SLAVE.quickSETUP();

		MASTER.init_LOG();
		SLAVE.init_LOG();
	}

	void INIT_LOWLEVELnUDP() {
		//MASTER.initialize_torque_control();
		//SLAVE.initialize_torque_control();

		MASTER.setLOWLEVEL();
		SLAVE.setLOWLEVEL();

		// Initialising low level actuator by actuator instead of manipulator by manipulator

		for (i = 0; i < MASTER.ACTUATOR_COUNT; i++)
		{
			// Save the current actuator position, to avoid a following error
			MASTER.base_command.add_actuators()->set_position(MASTER.base_feedback.actuators(i).position());
			SLAVE.base_command.add_actuators()->set_position(SLAVE.base_feedback.actuators(i).position());
			
			MASTER.q[i] = (MASTER.base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			MASTER.qd[i] = MASTER.base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
			MASTER.ROB_CMD.des_q[i] = MASTER.q[i]; // initialising the des_q to the home configuration you start from - VERY IMPORTANT
			MASTER.ROB_CMD.tau_ctrl[i] = 0;

			SLAVE.q[i] = (SLAVE.base_feedback.actuators(i).position() * rl::math::DEG2RAD);
			SLAVE.qd[i] = SLAVE.base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
			SLAVE.ROB_CMD.des_q[i] = SLAVE.q[i]; // initialising the des_q to the home configuration you start from - VERY IMPORTANT
			SLAVE.ROB_CMD.tau_ctrl[i] = 0;
		}

		// Initialise the gripper position, velocity and force
		if (MASTER.ROB_PARAMS.gripper_val)
		{
			float gripper_initial_position = MASTER.base_feedback.interconnect().gripper_feedback().motor()[0].position();
			MASTER.base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
			MASTER.m_gripper_motor_command = MASTER.base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
			MASTER.m_gripper_motor_command->set_position(gripper_initial_position);
			MASTER.m_gripper_motor_command->set_velocity(0.0);
			MASTER.m_gripper_motor_command->set_force(100.0);
		}


		// Initialise the gripper position, velocity and force
		if (SLAVE.ROB_PARAMS.gripper_val)
		{
			float gripper_initial_position = SLAVE.base_feedback.interconnect().gripper_feedback().motor()[0].position();
			SLAVE.base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);
			SLAVE.m_gripper_motor_command = SLAVE.base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
			SLAVE.m_gripper_motor_command->set_position(gripper_initial_position);
			SLAVE.m_gripper_motor_command->set_velocity(0.0);
			SLAVE.m_gripper_motor_command->set_force(100.0);
		}

		std::cout << "Sending first frame" << std::endl;
		// Send a first frame
		MASTER.base_feedback_async = MASTER.base_cyclic->Refresh_async(MASTER.base_command);
		SLAVE.base_feedback_async = SLAVE.base_cyclic->Refresh_async(SLAVE.base_command);


		std::cout << "Change Control Mode - Torque" << std::endl;
		std::cout << "Starting Gravity Compensation" << std::endl;

		// Set actuator in torque mode now that the command is equal to measure
		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
		for (i = 0; i < MASTER.ACTUATOR_COUNT; i++)
		{
			if (i >= MASTER.ROB_LOG.focus)
			{
				MASTER.actuator_config->SetControlMode(control_mode_message, i + 1);
				SLAVE.actuator_config->SetControlMode(control_mode_message, i + 1);
			}
		}


		//MASTER.init_TORQUE_CONTROL();
		//SLAVE.init_TORQUE_CONTROL();

		MASTER.init_UDP();
		SLAVE.init_UDP();
	}

	void TELEOP_GETFBK() {
		MASTER.getFBK();
		SLAVE.getFBK();
	}

	void TELEOP_LOGIT() {
		MASTER.LOG_IT();
		SLAVE.LOG_IT();
	}

	void TELEOP_sendCMD() {
		MASTER.sendCMD();
		SLAVE.sendCMD();
	}

	void CLEANUP() {
		// As opposed to using myKINOVA::myCLEANUP(); - the iterative shutdown of the actuators manipulator by manipulator takes time
		// Instead one should ensure the resetservoing mode is done actuator by actuator for all manipulators, and NOT manipulator by manipulator.
		SLAVE.ROB_LOG.data_count_final = SLAVE.ROB_LOG.data_count;
		MASTER.ROB_LOG.data_count_final = MASTER.ROB_LOG.data_count;
		
		std::cout << "Ending Gravity Compensation" << std::endl;
		std::cout << "Change Control Mode - Position" << std::endl;

		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

		for (i = 0; i < MASTER.ACTUATOR_COUNT; i++)
		{
			MASTER.actuator_config->SetControlMode(control_mode_message, i + 1);
			SLAVE.actuator_config->SetControlMode(control_mode_message, i + 1);
		}

		std::cout << "Setting single level servoing" << std::endl;
		//Set the servoing mode back to Single Level
		MASTER.servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
		SLAVE.servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
		MASTER.base->SetServoingMode(MASTER.servoing_mode);
		SLAVE.base->SetServoingMode(SLAVE.servoing_mode);

		// UDP clean up
		MASTER.ROB_UDP.cleanup();
		SLAVE.ROB_UDP.cleanup();

		// write logs
		MASTER.myWRITE();
		SLAVE.myWRITE();
	}

	myTELEOP() {// default constructors

	}

	myTELEOP(myPARAMS M_PARAMS_IN, myPARAMS S_PARAMS_IN) {
		master_PARAMS = M_PARAMS_IN;
		slave_PARAMS = S_PARAMS_IN;
		MASTER = getROB_OBJ(master_PARAMS, 0); // role = 0 for master - VERY IMPORTANT
		SLAVE = getROB_OBJ(slave_PARAMS, 1); // role = 1 for slave - VERY IMPORTANT
	}

	myTELEOP(myPARAMS M_PARAMS_IN, myPARAMS S_PARAMS_IN, float k_in) {
		master_PARAMS = M_PARAMS_IN;
		slave_PARAMS = S_PARAMS_IN;
		MASTER = getROB_OBJ_K(master_PARAMS, 0, k_in); // role = 0 for master - VERY IMPORTANT
		SLAVE = getROB_OBJ_K(slave_PARAMS, 1, k_in); // role = 1 for slave - VERY IMPORTANT
	}
};