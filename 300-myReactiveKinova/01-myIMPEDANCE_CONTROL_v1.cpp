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
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include <conio.h>
#include <time.h>
#include <ctime>

// Includes for converting data type from float to string
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <conio.h>
#include <sys/stat.h>
#include <io.h>
#include <inttypes.h>
#include <windows.h>
#include <process.h>

// Includes for UDP - start
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define SEND_PORT 27015
#define RECV_PORT 27016

// Include for RECV
#ifndef UNICODE
#define UNICODE
#endif

#include <iostream>

#include <sys/types.h>
// Includes for UDP - end


#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

#define ACTUATOR_COUNT 7
#define DURATION 400// Network timeout (seconds)
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);
float time_duration = DURATION; // Duration of the example (seconds)
namespace k_api = Kinova::Api;

double time_log[DURATION * 1000]{};
k_api::BaseCyclic::Feedback data_log[DURATION * 1000]{};

int focus = 0;

int64_t GetTickUs()
{
#if defined(_MSC_VER)
	LARGE_INTEGER start, frequency;

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);

	return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);

	return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

void write_to_file_g(k_api::BaseCyclic::Feedback data_log[], const std::string robot_model, rl::math::Vector del_q, rl::math::Vector b, double time_log[], int duration, int data_count, time_t now, string file_name)
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

	std::cout << "Writing to file..." << std::endl;
	string base_path = "D:/KINOVA/api_cpp/examples/logs/";
	string file = file_name;
	string timestamp = GetTimestamp(now);

	ofstream log_file(base_path + file + timestamp);
	if (log_file.is_open())
	{
		log_file << "Time(ms),Time(s),"
			<< GetSymbol_JointDamping() << ","
			<< GetSymbol_JointOffset() << ","
			<< GetSymbol_JointAngles('f') << ","
			<< GetSymbol_JointVelocity('f') << ","
			<< GetSymbol_JointTorque('f') << ","
			<< GetSymbol_JointTorque('g') << ","
			<< GetSymbol_JointTorque('v') << ","
			<< GetSymbol_JointTorque('m') << ","
			<< GetSymbol_JointTorque('b') << ","
			<< GetSymbol_JointTorque('c') << ","
			<< ",Index\n";

		for (int i = 0; i < duration * 1000 && i < data_count - 1; ++i)
		{
			log_file << (time_log[i] - time_log[0]) / 1000 << ",";
			log_file << (time_log[i] - time_log[0]) / 1000000 << ",";

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << b[j] << ",";
			}

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << del_q[j] << ",";
			}

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).position() * rl::math::DEG2RAD << ",";
				q[j] = (data_log[i].actuators(j).position() * rl::math::DEG2RAD) + del_q[j];
			}

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).velocity() * rl::math::DEG2RAD << ",";
				qd[j] = data_log[i].actuators(j).velocity() * rl::math::DEG2RAD;
			}

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << data_log[i].actuators(j).torque() << ",";
			}

			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->setAcceleration(qdd);
			dynamics->calculateGravity();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getGravity()[j] << ",";
			}

			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->setAcceleration(qdd);
			dynamics->calculateCentrifugalCoriolis();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getCentrifugalCoriolis()[j] << ",";
			}

			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->setAcceleration(qdd);
			dynamics->inverseDynamics();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << dynamics->getTorque()[j] << ",";
			}

			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << (b[j] * qd[j]) << ",";
			}

			dynamics->setPosition(q);
			dynamics->setVelocity(qd);
			dynamics->setAcceleration(qdd);
			dynamics->inverseDynamics();
			for (int j = 0; j < ACTUATOR_COUNT; ++j)
			{
				log_file << (dynamics->getTorque()[j] + (b[j] * qd[j])) << ",";
			}

			log_file << i << "\n";
		}
	}
	std::cout << "Writing to file completed!" << std::endl;
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


bool gravity_compensation_async(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, const std::string robot_model, bool gripper)
{
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
	rl::math::Vector B_gain(dynamics->getDof());
	rl::math::Vector B_net(dynamics->getDof());
	rl::math::Vector K(dynamics->getDof());
	rl::math::Vector del_q(dynamics->getDof());
	std::cout << "im here2" << std::endl;
	std::cout << dynamics->getDof() << endl;

	rl::math::Vector UDP_q(dynamics->getDof() + 1);
	rl::math::Vector des_q(dynamics->getDof() + 1);

	q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	B << 11.044, 11.428, 10.415, 10.965, 9.4533, 9.5262, 9.5115;
	B_gain << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.8;
	B_net << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	K << 15, 15, 15, 15, 15, 15, 15;
	del_q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//del_q << 0.0044, 0.0088, -0.0035, -0.0043, 0.0068, 0.0026, -0.0084;


	rl::math::Vector dq(dynamics->getDof());
	int K_new = 10; // ### TUNE THIS ### //
	dq << 0.0073, 0.0184, -0.0055, 0.0041, 0.006, 0.0017, -0.0084;
	K << K_new, K_new, K_new, K_new, K_new, K_new, K_new; //Joint Impedance Stiffness

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
	// Set up UDP

	//----------------------
	// Declare and initialize variables - SEND
	WSADATA wsaData;
	int iResult;

	SOCKET ConnectSocket = INVALID_SOCKET;
	struct sockaddr_in clientService;
	//int SenderAddrSize = sizeof(clientService);

	char* sendbuf = "this is a test";
	char recvbuf[DEFAULT_BUFLEN];
	int recvbuflen = DEFAULT_BUFLEN;

	// Declare and initialize variables - RECV
	int iResult2 = 0;

	SOCKET RecvSocket;
	struct sockaddr_in RecvAddr;

	unsigned short Port2 = 27016;

	char RecvBuf[1024];
	int BufLen = 1024;

	struct sockaddr_in SenderAddr;
	int SenderAddrSize = sizeof(SenderAddr);

	//----------------------
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR) {
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}

	//----------------------
	// Create a SOCKET for connecting to server - SEND
	ConnectSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (ConnectSocket == INVALID_SOCKET) {
		printf("Error at socket(): %ld\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}

	// Create a receiver socket to receive datagrams - RECV
	RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	//struct timeval read_timeout;
	//read_timeout.tv_sec = 0;
	//read_timeout.tv_usec = 10;
	//setsockopt(RecvSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&read_timeout, sizeof read_timeout);
	int ret, iVal = 1;
	unsigned int  sz = sizeof(iVal);
	iVal = 1; // if you set this too low, the recvsocket will be impatient and might not 
	ret = setsockopt(RecvSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&iVal, sz);

	if (RecvSocket == INVALID_SOCKET) {
		wprintf(L"socket failed with error %d\n", WSAGetLastError());
		return 1;
	}

	//----------------------
	// The sockaddr_in structure specifies the address family,
	// IP address, and port of the server to be connected to. - SEND
	clientService.sin_family = AF_INET;
	clientService.sin_addr.s_addr = inet_addr("127.0.0.1");
	clientService.sin_port = htons(SEND_PORT);

	// Bind the socket to any address and the specified port - RECV
	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	RecvAddr.sin_port = htons(RECV_PORT);

	iResult2 = ::bind(RecvSocket, (SOCKADDR*)&RecvAddr, sizeof(RecvAddr));
	if (iResult2 != 0) {
		wprintf(L"bind failed with error %d\n", WSAGetLastError());
		return 1;
	}

	printf("Connect instead of bind was run.\n");
	if (iResult2 == -1) {
		printf("Connect instead of bind failed.\n");
		exit(2);
	}

	//----------------------
	// Connect to server.
	iResult = connect(ConnectSocket, (SOCKADDR*)&clientService, sizeof(clientService));
	if (iResult == SOCKET_ERROR) {
		closesocket(ConnectSocket);
		printf("Unable to connect to server: %ld\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}

	// Send an initial buffer
	iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		return 1;
	}

	printf("Bytes Sent: %ld\n", iResult);

	try
	{


		// Initiate low level Kinova control

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
			UDP_q[i] = q[i]; // initialising the UDP_q to the home configuration you start from - VERY IMPORTANT
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

		// Defining the UDP send
		float num_TOSEND[7];
		num_TOSEND[0] = 0;
		num_TOSEND[1] = 0;
		num_TOSEND[2] = 0;
		num_TOSEND[3] = 0;
		num_TOSEND[4] = 0;
		num_TOSEND[5] = 0;
		num_TOSEND[6] = 0;

		//int num_TOSEND_idx = 0;
		std::string sendbuf1;
		std::string sendbuf2;
		sendbuf2.append("q_start");
		//= "STRING_TO_SEND:";
		for (int num_TOSEND_idx = 0; num_TOSEND_idx < 7; num_TOSEND_idx++) {
			sendbuf1 = std::to_string(num_TOSEND[num_TOSEND_idx]);
			sendbuf2.append(sendbuf1);
			sendbuf2.append("||");
		}
		sendbuf2.append("q_end");
		//std::cout << sendbuf2 << std::endl;
		const char* sendbuf3 = sendbuf2.c_str();

		int delimiter_idx = 0;

		//Realtime control loop. Press "Q" key to exit loop.
		while (!(GetKeyState('Q') & 0x8000))
		{
			now = GetTickUs();
			if (now - last >= 1000)
			{
				//-----------------------------------------------------------------------------//
				base_feedback = base_feedback_async.get();

				if (timer_count % 1 == 0 && data_count < DURATION * 1000 && logging == 1)
				{
					data_log[data_count] = base_feedback;
					time_log[data_count] = GetTickUs();
					++data_count;
				}

				for (i = 0; i < ACTUATOR_COUNT; i++)
				{
					q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD) + del_q[i];
					qd[i] = base_feedback.actuators(i).velocity() * rl::math::DEG2RAD;
					qdd[i] = 0.0;
					num_TOSEND[i] = q[i];
					des_q[i] = UDP_q[i];
				}

				dynamics->setPosition(q);
				dynamics->setVelocity(qd);
				dynamics->setAcceleration(qdd);
				dynamics->inverseDynamics();

				for (i = 0; i < ACTUATOR_COUNT; ++i)
				{
					if (i >= focus)
					{
						base_command.mutable_actuators(i)->set_torque_joint(dynamics->getTorque()[i] + (B_gain[i] * B[i] * qd[i]) +(K[i] * calculate_delta_q(des_q[i] - dq[i], q[i], 'r')));
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


				// UDP part - start
				std::string sendbuf2;
				sendbuf2.append("q_start");
				//= "STRING_TO_SEND:";
				for (int num_TOSEND_idx = 0; num_TOSEND_idx < 7; num_TOSEND_idx++) {
					sendbuf1 = std::to_string(num_TOSEND[num_TOSEND_idx]);
					sendbuf2.append(sendbuf1);
					sendbuf2.append("||");
				}
				sendbuf2.append("q_end");
				const char* sendbuf3 = sendbuf2.c_str();
				wprintf(L"Sending datagrams...\n");

				iResult = send(ConnectSocket, sendbuf3, (int)strlen(sendbuf3), 0);
				wprintf(L"datagrams sent...\n");
				iResult2 = recvfrom(RecvSocket,
					RecvBuf, BufLen, 0, (SOCKADDR*)&SenderAddr, &SenderAddrSize);

				std::cout << "At recvfrom" << std::endl;
				if (iResult2 > 0) // print recvbuffer ONLY if something was received
				{
					wprintf(L"Received datagrams...\n");

					std::cout << RecvBuf << std::endl;
					string myMATLAB_DATA(RecvBuf);
					std::string delimiter = "||";
					size_t pos = 0;
					std::string token;
					delimiter_idx = 0;
					while ((pos = myMATLAB_DATA.find(delimiter)) != std::string::npos) {
						token = myMATLAB_DATA.substr(0, pos);

						float num_float = std::stof(token);

						UDP_q[delimiter_idx] = num_float;

						myMATLAB_DATA.erase(0, pos + delimiter.length());
						delimiter_idx++;
						if (delimiter_idx > 6)
						{
							break;
						}
					}

					std::cout << "q1 =" << UDP_q[0] << " q2 =" << UDP_q[1] << " q3 =" << UDP_q[2] << " q4 =" << UDP_q[3] << " q5 =" << UDP_q[4] << " q6 =" << UDP_q[5] << " q7 =" << UDP_q[6] << std::endl;

				}

				std::cout << "des q1 =" << des_q[0] << " q2 =" << des_q[1] << " q3 =" << des_q[2] << " q4 =" << des_q[3] << " q5 =" << des_q[4] << " q6 =" << des_q[5] << " q7 =" << des_q[6] << std::endl;

				// UDP part - end

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
					std::cout << "At Refresh_async" << std::endl;
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
	time_t now_t = time(0);
	string file_name = "gravity_log_file_";
	for (int k = 0; k < ACTUATOR_COUNT; k++) {
		B_net[k] = B_gain[k] * B[k];
	}

	// UDP close - start
	// cleanup - SEND
	closesocket(ConnectSocket);

	// Close the socket when finished receiving datagrams - RECV
	wprintf(L"Finished receiving. Closing socket.\n");
	iResult2 = closesocket(RecvSocket);


	WSACleanup();
	// UDP close - end

	write_to_file_g(data_log, robot_model, del_q, B_net, time_log, time_duration, data_count, now_t, file_name);

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

	// ### TO DO ###: Change to correct IP
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
	const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_GRIPPER_2024.urdf";

	//example_move_to_home_position(base);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout << "Initializing gravity compensation" << std::endl;

	// ### TO DO ###: Use correct robot model and gripper configuration
	gravity_compensation_async(base, base_cyclic, actuator_config, robot_model, TRUE);

	return 0;
}