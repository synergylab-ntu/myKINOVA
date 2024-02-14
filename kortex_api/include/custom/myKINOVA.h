

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

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

namespace k_api = Kinova::Api;

class myKINOVA {

	int ACTUATOR_COUNT = 7;

public:
	float q[7];

	/*void getPOS() {
		for (int i = 0; i < ACTUATOR_COUNT; i++)
		{
			q[i] = (base_feedback.actuators(i).position() * rl::math::DEG2RAD);
		}
	}*/

	//void ROBOT_SETUP() {
	//	bool return_status = true;

	//// Clearing faults
	//try
	//{
	//	base->ClearFaults();
	//}
	//catch (...)
	//{
	//	std::cout << "Unable to clear robot faults" << std::endl;
	//	return false;
	//}

	//rl::mdl::UrdfFactory factory;
	//std::cout << "im here" << std::endl;

	//
	//std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model));
	//std::cout << "mode here" << model.get() << std::endl;


	//rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());
	//rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());


	//rl::math::Vector q(dynamics->getDof());
	//rl::math::Vector qd(dynamics->getDof());
	//rl::math::Vector qdd(dynamics->getDof());
	//rl::math::Vector B(dynamics->getDof());
	//rl::math::Vector K(dynamics->getDof());
	//rl::math::Vector des_q(dynamics->getDof());
	//rl::math::Vector tau_cmd(dynamics->getDof());
	//rl::math::Vector ext_tau(dynamics->getDof());
	//rl::math::Vector ext_tau_limit(dynamics->getDof());
	//std::cout << "im here2" << std::endl;
	//std::cout << dynamics->getDof() << endl;

	//des_q << 0, 0, 0, 0, 0, 0, 0;
	//tau_cmd << 0, 0, 0, 0, 0, 0, 0;
	//ext_tau << 0, 0, 0, 0, 0, 0, 0;
	//ext_tau_limit << 3, 2, 2, 5, 4, 3, 6;
	//q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//B << 5.5220,5.7140,5.2075,5.4825,4.7267,4.7631,7.6092;
	//K << 10, 10, 10, 10, 10, 10, 10;

	//std::cout << "im here3" << std::endl;

	//dynamics->setPosition(q);
	//dynamics->setVelocity(qd);
	//dynamics->setAcceleration(qdd);


	//k_api::BaseCyclic::Feedback base_feedback;
	//k_api::BaseCyclic::Command  base_command;
	//k_api::GripperCyclic::MotorCommand* m_gripper_motor_command;

	//std::future<k_api::BaseCyclic::Feedback> base_feedback_async;

	//auto servoing_mode = k_api::Base::ServoingModeInformation();

	//myKINOVA_LOG ROBOT_LOG(DURATION);
	//
	//int i = 0;
	//int timer_count = 0;
	//int data_count = 0;
	//int64_t now = 0;
	//int64_t last = 0;
	//float actual_position;
	//int logging = 1;      //change to zero to manually start logging

	//}

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

	bool goHOME(k_api::Base::BaseClient* base)
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

	myKINOVA(std::string robot_model_in) {
		//std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model_in));
	}

};
