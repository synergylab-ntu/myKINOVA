#include <Logging.h>
#include <KinovaInit.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include <conio.h>
#include <time.h>
#include <ctime>
#include <sstream> // std::stringstream

#include <chrono>
using namespace std::chrono;
int64_t timestamp = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define ACTUATOR_COUNT 7
#define DURATION 2000// Network timeout (seconds)
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);
float time_duration = DURATION; // Duration of the example (seconds)
namespace k_api = Kinova::Api;

double time_log[DURATION * 1000]{};
int64_t unix_epoch[DURATION * 1000]{};
k_api::BaseCyclic::Feedback data_log[DURATION * 1000]{};
std::vector<float> trajectory_log[DURATION * 1000]{};

int focus = 0;


void replace_all(
	std::string& s,
	std::string const& toReplace,
	std::string const& replaceWith
) {
	std::string buf;
	std::size_t pos = 0;
	std::size_t prevPos;

	// Reserves rough estimate of final size of string.
	buf.reserve(s.size());

	while (true) {
		prevPos = pos;
		pos = s.find(toReplace, pos);
		if (pos == std::string::npos)
			break;
		buf.append(s, prevPos, pos - prevPos);
		buf += replaceWith;
		pos += toReplace.size();
	}

	buf.append(s, prevPos, s.size() - prevPos);
	s.swap(buf);
}


int64_t GetTickUs()
{
#if defined(_MSC_VER)
	LARGE_INTEGER start, frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);

	timestamp = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
	//cout << timestamp << std::endl;

	return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);

	return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

std::vector<std::vector<float>> read_csv(std::string filename) {

	// Create a vector of <int vector> pairs to store the result
	std::vector<std::vector<float>> result;

	// Create an input filestream
	std::ifstream myFile(filename);

	// Make sure the file is open
	if (!myFile.is_open()) throw std::runtime_error("Could not open file");

	// Helper vars
	std::string line, colname;
	float val;

	// Read the column names
	if (myFile.good()) {
		// Read data, line by line
		while (std::getline(myFile, line)) {
			// Create a stringstream of the current line
			std::stringstream ss(line);

			// Keep track of the current column index
			int colIdx = 0;
			std::vector<float> temp_result;

			// Extract each integer
			while (ss >> val) {
				// Add the current integer to the 'colIdx' column's values vector
				temp_result.push_back(val);

				// If the next token is a comma, ignore it and move on
				if (ss.peek() == ',') ss.ignore();

				// Increment the column index
				colIdx++;
			}
			result.push_back(temp_result);
		}
	}

	// Close file
	myFile.close();

	return result;
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

void write_to_file_g(
	k_api::BaseCyclic::Feedback data_log[],
	const std::string robot_model,
	std::vector<float> trajectory_log[],
	rl::math::Vector K,
	rl::math::Vector B,
	rl::math::Vector threshold,
	int64_t unix_epoch[],
	double time_log[],
	int duration,
	time_t now,
	string file_name,
	int data_count
)
{
	rl::mdl::UrdfFactory factory;
	std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model));

	rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());
	rl::math::Vector q(dynamics->getDof());
	rl::math::Vector qd(dynamics->getDof());
	rl::math::Vector qdd(dynamics->getDof());
	q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	dynamics->setPosition(q);
	dynamics->setVelocity(qd);
	dynamics->setAcceleration(qdd);
	dynamics->inverseDynamics();

	std::cout << "Writing to file..." << std::endl;
	string timestamp = GetTimestamp(now);
	int stiffnessValue = K[0];
	ofstream log_file(file_name);
	if (log_file.is_open())
	{
		log_file << "Time(ms),Time(s),Unix_epoch(ns)," << "K_1,K_2,K_3,K_4,K_5,K_6,K_7," << "B_1,B_2,B_3,B_4,B_5,B_6,B_7,";
		log_file << GetSymbol_JointAngles('f') << "," << "q_des_1,q_des_2,q_des_3,q_des_4,q_des_5,q_des_6,q_des_7,";
		log_file << GetSymbol_JointVelocity('f') << "," << "qd_des_1,qd_des_2,qd_des_3,qd_des_4,qd_des_5,qd_des_6,qd_des_7,";
		log_file << GetSymbol_JointTorque('f') << "," << GetSymbol_JointTorque('c') << "," << GetSymbol_JointTorque('g') << "," << GetSymbol_JointTorque('m');
		log_file << ",Index\n";

		for (int i = 0; i < duration * 1000 && i < data_count - 1; ++i)
		{
			log_file << (time_log[i] - time_log[0]) / 1000 << ",";
			log_file << (time_log[i] - time_log[0]) / 1000000 << ",";
			//log_file << std::fixed << std::setprecision(20) << 
			log_file << unix_epoch[i] << ",";
			//log_file << std::fixed << std::setprecision(25) << unix_epoch[i] << ",";

			// K
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << K[j] << ",";
			}

			// B
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << B[j] << ",";
			}

			// q_fbk
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).position() * rl::math::DEG2RAD << ",";
				q[j] = data_log[i].actuators(j).position() * rl::math::DEG2RAD;
			}

			// q_des
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << trajectory_log[i][j] << ",";
			}

			// qd_fbk
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).velocity() * rl::math::DEG2RAD << ",";
				qd[j] = data_log[i].actuators(j).velocity() * rl::math::DEG2RAD;
			}

			// qd_des
			for (int j = 7; j < ACTUATOR_COUNT + 7; ++j)
			{
				log_file << trajectory_log[i][j] << ",";
			}

			// tau_fbk
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).torque() << ",";
			}

			// tau_cmd
			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->inverseDynamics();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getTorque()[j] + K[j] * calculate_delta_q(trajectory_log[i][j], q[j], 'r') << ",";
			}

			// tau_g
			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->calculateGravity();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getGravity()[j] << ",";
			}

			// tau_model
			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->inverseDynamics();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getTorque()[j] << ",";
			}

			log_file << i << "\n";
		}
	}
	std::cout << "Writing to file completed!" << std::endl;
}

bool trajectory_impedance_async(
	k_api::Base::BaseClient* base,
	k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
	k_api::ActuatorConfig::ActuatorConfigClient* actuator_config,
	const std::string robot_model,
	bool gripper,
	std::string trajectory_folder,
	std::string exp_folder,
	std::string trajectory_files[],
	std::string trajectory_logfile

)
{
	bool return_status = true;
	//int K_new = 400; // ### TUNE THIS ### //
	int K_new = 100; // ### TUNE THIS ### //
	//int K_new = 15; // stiffness used in the teleoperation code
	float th = 0.05; // ### TUNE THIS ### // // was 0.015, then 0.055
	int delay = 2000; //delay between trajectory playback in ms

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
	std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model));

	rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());

	rl::math::Vector q(dynamics->getDof());
	rl::math::Vector qd(dynamics->getDof());
	rl::math::Vector dq(dynamics->getDof());
	rl::math::Vector qdd(dynamics->getDof());
	rl::math::Vector B(dynamics->getDof());
	rl::math::Vector B_gain(dynamics->getDof());
	rl::math::Vector B_net(dynamics->getDof());
	rl::math::Vector K(dynamics->getDof());
	rl::math::Vector threshold(dynamics->getDof());
	rl::math::Vector del_q(dynamics->getDof());

	q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//dq << 0,0,0,0,0,0,0;
	//dq << 0.0178, 0.0313, -0.0006, 0.0197, 0.0112, -0.0092, 0.0049; // MOCAP_RECAL.dq
	dq << 0.0075, 0.0146, -0.0030, 0.0022, 0.0068, -0.0014, -0.0053; // ORACLE_RECAL.dq
	qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	B << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//B << 11.044, 11.428, 10.415, 10.965, 9.4533, 9.5262, 9.5115;
	B_gain << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.8;
	B_net << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	del_q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//del_q << 0.0044, 0.0088, -0.0035, -0.0043, 0.0068, 0.0026, -0.0084;

	// ### TUNE THIS ### //
	K << K_new, K_new, K_new, K_new, K_new, K_new, K_new; //Joint Impedance Stiffness
	threshold << th, th, th, th, th, th, th; //Joint Angle error threshold

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

	try
	{
		//Read 4 CSVs with trajectories [waypoint joint angels (rad)]
		std::vector<std::vector<std::vector<float>>> trajectory_angles;
		//for (int i = 0; i < 1; ++i) {
		std::cout << trajectory_folder + trajectory_files[0] << std::endl;
		trajectory_angles.push_back(read_csv(trajectory_folder + trajectory_files[0]));
		//}
		int trajectory = 0;
		int end_trajectory = trajectory_angles.size() - 1;
		int waypoint = 0;
		int end_waypoint = trajectory_angles[trajectory].size() - 1;
		int transition_delay = 0;

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
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD) + del_q[i];
			qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
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
		std::cout << "Starting Trajectory Impedance Control" << std::endl;
		std::cout << "'Q': Safely Quit Execution of Program" << std::endl;

		// Set actuator in torque mode now that the command is equal to measure
		auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
		control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
		for (i = 0; i < ACTUATOR_COUNT; i++)
		{
			actuator_config->SetControlMode(control_mode_message, i + 1);
		}

		//Realtime control loop. Press "Q" key to exit loop.
		while (!(GetKeyState('Q') & 0x8000))
		{
			now = GetTickUs();
			if (now - last >= 1000)

			{
				//-----------------------------------------------------------------------------//
				base_feedback = base_feedback_async.get();

				//Data Logging
				if (timer_count % 1 == 0 && data_count < DURATION * 1000)
				{
					data_log[data_count] = base_feedback;
					time_log[data_count] = GetTickUs();
					unix_epoch[data_count] = timestamp;
					//cout << unix_epoch[data_count] << std::endl;
					trajectory_log[data_count] = trajectory_angles[trajectory][waypoint];
					++data_count;
				}

				//Reading feedback
				for (i = 0; i < ACTUATOR_COUNT; i++)
				{
					q[i] = base_feedback.actuators(i).position() * rl::math::DEG2RAD;
					qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
					qdd[i] = 0.0;
				}

				//Calculating gravity compensation torque
				dynamics->setPosition(q);
				dynamics->setVelocity(qd);
				dynamics->setAcceleration(qdd);
				dynamics->inverseDynamics();


				//Checking if all jont angle errors are within threshold
				int err_check = 0;
				for (i = 0; i < ACTUATOR_COUNT; ++i) {
					if (abs(calculate_delta_q(trajectory_angles[trajectory][waypoint][i], q[i], 'r')) > threshold[i]) {
						++err_check;
					}
				}

				//If all angles are within threshold, update to next waypoint in trajectory
				if (err_check == 0) {
					//Check that end has not been reached
					if (waypoint < end_waypoint) {
						++waypoint;
						cout << "iteration number is" << waypoint << std::endl;
					}
				}

				for (i = 0; i < ACTUATOR_COUNT; ++i)
				{
					base_command.mutable_actuators(i)->set_torque_joint(dynamics->getTorque()[i] + (K[i] * calculate_delta_q(trajectory_angles[trajectory][waypoint][i] - dq[i], q[i], 'r')));
					//cout << "dtheta = " << calculate_delta_q(trajectory_angles[trajectory][waypoint][i], q[i], 'r') << std::endl;
					base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
				}

				actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
				if (waypoint == end_waypoint) {
					if (trajectory == 0) {
						m_gripper_motor_command->set_position(50.0);
						m_gripper_motor_command->set_velocity(1.0);
						m_gripper_motor_command->set_force(1.0);
					}
					else if (trajectory == 2) {
						m_gripper_motor_command->set_position(0.0);
						m_gripper_motor_command->set_velocity(100.0);
						m_gripper_motor_command->set_force(0.0);
					}

					if (trajectory != end_trajectory) {
						if (transition_delay < delay)
							++transition_delay;
						else {
							transition_delay = 0;
							++trajectory;
							waypoint = 0;
							end_waypoint = trajectory_angles[trajectory].size() - 1;
						}
					}
				}
				//-----------------------------------------------------------------------------//
				//-------------------------- Gripper commands ----------------------------------------------------------

				actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
				if (GetKeyState('W') & 0x8000)  //close gripper
				{
					if (actual_position + 10.0 > 100.0)
						m_gripper_motor_command->set_position(100.0);
					else
						m_gripper_motor_command->set_position(actual_position + 10.0);
					//m_gripper_motor_command->set_position(14.0); // for a fixed know position
					m_gripper_motor_command->set_velocity(40.0);
					m_gripper_motor_command->set_force(100.0);
				}
				if (GetKeyState('S') & 0x8000)   //open gripper
				{
					if (actual_position - 10.0 < 0.0)
						m_gripper_motor_command->set_position(0.0);
					else
						//m_gripper_motor_command->set_position(actual_position - 10.0);
						m_gripper_motor_command->set_position(0.0); //fully opens gripper
					m_gripper_motor_command->set_velocity(40.0);
					m_gripper_motor_command->set_force(1.0);
				}

				m_gripper_motor_command->set_position(trajectory_angles[trajectory][waypoint][7]); // Gripper command to be taken from the Trajectory file using GMM - Keep as the 8th entry in the csv file.
				m_gripper_motor_command->set_velocity(40.0);
				m_gripper_motor_command->set_force(100.0);

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
				last = GetTickUs();
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

	std::cout << "Ending Trajectory Impedance Control" << std::endl;

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
	time_t now_t = time(0);
	string file_name = exp_folder + trajectory_logfile;
	for (int k = 0; k < ACTUATOR_COUNT; k++) {
		B_net[k] = B_gain[k] * B[k];
	}
	write_to_file_g(data_log, robot_model, trajectory_log, K, B_net, threshold, unix_epoch, time_log, time_duration, now_t, file_name, data_count);

	// Wait for a bit
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	return return_status;
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

bool example_move_to_home_position(k_api::Base::BaseClient* base)
{
	// Make sure the arm is in Single Level Servoing before executing an Action
	auto servoingMode = k_api::Base::ServoingModeInformation();
	servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
	base->SetServoingMode(servoingMode);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Move arm to ready position
	std::cout << "Moving the arm to a safe position" << std::endl;
	auto action_type = k_api::Base::RequestedActionType();
	action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
	auto action_list = base->ReadAllActions(action_type);
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
		const auto notification_handle = base->OnNotificationActionTopic(
			create_action_event_listener_by_promise(promise),
			k_api::Common::NotificationOptions{}
		);

		base->ExecuteActionFromReference(action_handle);

		// Wait for action to finish
		const auto status = future.wait_for(TIMEOUT_DURATION);
		base->Unsubscribe(notification_handle);

		if (status != std::future_status::ready)
		{
			std::cout << "Timeout on action notification wait" << std::endl;
			return false;
		}

		return true;

	}
}

int main()
{
	// Create API objects
	auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };

	KINOVA* slave_tcp = new KINOVA("192.180.0.107", 10000);
	slave_tcp->Init_TCP();

	KINOVA* slave_udp = new KINOVA("192.180.0.107", 10001);
	slave_udp->Init_UDP();

	std::cout << "Creating Client" << std::endl;
	auto base = new k_api::Base::BaseClient(slave_tcp->get_router());
	auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(slave_udp->get_router());
	auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(slave_tcp->get_router());
	std::cout << "Client Created" << std::endl;

	//Kinova with Robotiq 2f 85 Gripper
	const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_URDF_V12_fixed_Gripper_rev_2024.urdf";

	//std::string trajectory_folder = "D:/dropbox_harsha/Dropbox/_research/_____coding/2024-01-29_BiBReplays_v2/";
	std::string trajectory_folder = "C:/Users/Harsha/Dropbox/_research/_____coding/2024-01-29_BiBReplays_v2/";

	std::string exp_folder = trajectory_folder;
	std::string exp_settings = trajectory_folder;
	exp_settings.append("exp_settings.txt");


	std::string trajectory_files[1] = {
	"sens_grp_movements.csv"
	};

	std::string trajectory_logfile = "/kinova.csv";

	string read_exp_line;
	ifstream MyReadFile(exp_settings);
	getline(MyReadFile, read_exp_line);
	replace_all(read_exp_line, "\\", "/");
	read_exp_line.erase(0, 1);
	exp_folder.append(read_exp_line);
	exp_folder.append("/");
	std::cout << exp_folder << std::endl;
	example_move_to_home_position(base);
	std::cout << "Initializing trajectory impedance control" << std::endl;

	//trajectory_impedance_async(base, base_cyclic, actuator_config, calibrated_robot_model_gripper, TRUE, trajectory_folder, exp_folder, trajectory_files, trajectory_logfile); //replace with appropriate model and gripper configuration
	trajectory_impedance_async(base, base_cyclic, actuator_config, robot_model, TRUE, trajectory_folder, exp_folder, trajectory_files, trajectory_logfile); //replace with appropriate model and gripper configuration
	example_move_to_home_position(base);

	return 0;
}