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

//#include <Python.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <vector>
#include <conio.h>

#define ACTUATOR_COUNT 7

string GetTimestamp(time_t now) {
	tm* ltm = localtime(&now);
	string year = to_string(1900 + ltm->tm_year);
	string month = to_string(1 + ltm->tm_mon);
	string day = to_string(ltm->tm_mday);
	string hour = to_string(ltm->tm_hour);
	string min = to_string(ltm->tm_min);
	string sec = to_string(ltm->tm_sec);
	string timestamp = year + "_" + month + "_" + day + "_" + hour + min + sec + ".csv";
	return timestamp;
}

string GetSymbol_JointAngles(char type) {
	if (type == 'f')
		return "q_fbk_1,q_fbk_2,q_fbk_3,q_fbk_4,q_fbk_5,q_fbk_6,q_fbk_7";
	else if (type == 'c')
		return "q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6,q_cmd_7";
}

string GetSymbol_JointAngles_Master(char type) {
	if (type == 'f')
		return "master_q_fbk_1,master_q_fbk_2,master_q_fbk_3,master_q_fbk_4,master_q_fbk_5,master_q_fbk_6,master_q_fbk_7";
	else if (type == 'c')
		return "master_q_cmd_1,master_q_cmd_2,master_q_cmd_3,master_q_cmd_4,master_q_cmd_5,master_q_cmd_6,master_q_cmd_7";
}

string GetSymbol_JointAngles_Slave(char type) {
	if (type == 'f')
		return "slave_q_fbk_1,slave_q_fbk_2,slave_q_fbk_3,slave_q_fbk_4,slave_q_fbk_5,slave_q_fbk_6,slave_q_fbk_7";
	else if (type == 'c')
		return "slave_q_cmd_1,slave_q_cmd_2,slave_q_cmd_3,slave_q_cmd_4,slave_q_cmd_5,slave_q_cmd_6,slave_q_cmd_7";
}


string GetSymbol_JointVelocity(char type) {
	if (type == 'f')
		return "qd_fbk_1,qd_fbk_2,qd_fbk_3,qd_fbk_4,qd_fbk_5,qd_fbk_6,qd_fbk_7";
	else if (type == 'c')
		return "qd_cmd_1,qd_cmd_2,qd_cmd_3,qd_cmd_4,qd_cmd_5,qd_cmd_6,qd_cmd_7";
}

string GetSymbol_JointVelocity_Master(char type) {
	if (type == 'f')
		return "master_qd_fbk_1,master_qd_fbk_2,master_qd_fbk_3,master_qd_fbk_4,master_qd_fbk_5,master_qd_fbk_6,master_qd_fbk_7";
	else if (type == 'c')
		return "master_qd_cmd_1,master_qd_cmd_2,master_qd_cmd_3,master_qd_cmd_4,master_qd_cmd_5,master_qd_cmd_6,master_qd_cmd_7";
}

string GetSymbol_JointVelocity_Slave(char type) {
	if (type == 'f')
		return "slave_qd_fbk_1,slave_qd_fbk_2,slave_qd_fbk_3,slave_qd_fbk_4,slave_qd_fbk_5,slave_qd_fbk_6,slave_qd_fbk_7";
	else if (type == 'c')
		return "slave_qd_cmd_1,slave_qd_cmd_2,slave_qd_cmd_3,slave_qd_cmd_4,slave_qd_cmd_5,slave_qd_cmd_6,slave_qd_cmd_7";
}


string GetSymbol_JointTorque(char type) {
	if (type == 'f')
		return "tau_fbk_1,tau_fbk_2,tau_fbk_3,tau_fbk_4,tau_fbk_5,tau_fbk_6,tau_fbk_7";
	else if (type == 'g')
		return "tau_g_1,tau_g_2,tau_g_3,tau_g_4,tau_g_5,tau_g_6,tau_g_7";
	else if (type == 'v')
		return "tau_cor_1,tau_cor_2,tau_cor_3,tau_cor_4,tau_cor_5,tau_cor_6,tau_cor_7";
	else if (type == 'm')
		return "tau_model_1,tau_model_2,tau_model_3,tau_model_4,tau_model_5,tau_model_6,tau_model_7";
	else if (type == 'b')
		return "tau_b_1,tau_b_2,tau_b_3,tau_b_4,tau_b_5,tau_b_6,tau_b_7";
	else if (type == 'c')
		return "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7";
	else if (type == 'e')
		return "tau_ext_1,tau_ext_2,tau_ext_3,tau_ext_4,tau_ext_5,tau_ext_6,tau_ext_7";
	else if (type == 'o')
		return "tau_slave_interaction_1,tau_slave_interaction_2,tau_slave_interaction_3,tau_slave_interaction_4,tau_slave_interaction_5,tau_slave_interaction_6,tau_slave_interaction_7";
	else if (type == 'i')
		return "tau_slave_impedance_1,tau_slave_impedance_2,tau_slave_impedance_3,tau_slave_impedance_4,tau_slave_impedance_5,tau_slave_impedance_6,tau_slave_impedance_7";
}

string GetSymbol_JointTorque_Master(char type) {
	if (type == 'f')
		return "master_tau_fbk_1,master_master_tau_fbk_2,master_master_tau_fbk_3,master_master_tau_fbk_4,master_master_tau_fbk_5,master_master_tau_fbk_6,master_master_tau_fbk_7";
	else if (type == 'g')
		return "master_tau_g_1,master_tau_g_2,master_tau_g_3,master_tau_g_4,master_tau_g_5,master_tau_g_6,master_tau_g_7";
	else if (type == 'v')
		return "master_tau_cor_1,master_tau_cor_2,master_tau_cor_3,master_tau_cor_4,master_tau_cor_5,master_tau_cor_6,master_tau_cor_7";
	else if (type == 'm')
		return "master_tau_model_1,master_tau_model_2,master_tau_model_3,master_tau_model_4,master_tau_model_5,master_tau_model_6,master_tau_model_7";
	else if (type == 'b')
		return "master_tau_b_1,master_tau_b_2,master_tau_b_3,master_tau_b_4,master_tau_b_5,master_tau_b_6,master_tau_b_7";
	else if (type == 'c')
		return "master_tau_cmd_1,master_tau_cmd_2,master_tau_cmd_3,master_tau_cmd_4,master_tau_cmd_5,master_tau_cmd_6,master_tau_cmd_7";
	else if (type == 'e')
		return "master_tau_ext_1,master_tau_ext_2,master_tau_ext_3,master_tau_ext_4,master_tau_ext_5,master_tau_ext_6,master_tau_ext_7";
	else if (type == 'o')
		return "master_tau_slave_interaction_1,master_tau_slave_interaction_2,master_tau_slave_interaction_3,master_tau_slave_interaction_4,master_tau_slave_interaction_5,master_tau_slave_interaction_6,master_tau_slave_interaction_7";
	else if (type == 'i')
		return "master_tau_slave_impedance_1,master_tau_slave_impedance_2,master_tau_slave_impedance_3,master_tau_slave_impedance_4,master_tau_slave_impedance_5,master_tau_slave_impedance_6,master_tau_slave_impedance_7";
}

string GetSymbol_JointTorque_Slave(char type) {
	if (type == 'f')
		return "slave_tau_fbk_1,slave_tau_fbk_2,slave_tau_fbk_3,slave_tau_fbk_4,slave_tau_fbk_5,slave_tau_fbk_6,slave_tau_fbk_7";
	else if (type == 'g')
		return "slave_tau_g_1,slave_tau_g_2,slave_tau_g_3,slave_tau_g_4,slave_tau_g_5,slave_tau_g_6,slave_tau_g_7";
	else if (type == 'v')
		return "slave_tau_cor_1,slave_tau_cor_2,slave_tau_cor_3,slave_tau_cor_4,slave_tau_cor_5,slave_tau_cor_6,slave_tau_cor_7";
	else if (type == 'm')
		return "slave_tau_model_1,slave_tau_model_2,slave_tau_model_3,slave_tau_model_4,slave_tau_model_5,slave_tau_model_6,slave_tau_model_7";
	else if (type == 'b')
		return "slave_tau_b_1,slave_tau_b_2,slave_tau_b_3,slave_tau_b_4,slave_tau_b_5,slave_tau_b_6,slave_tau_b_7";
	else if (type == 'c')
		return "slave_tau_cmd_1,slave_tau_cmd_2,slave_tau_cmd_3,slave_tau_cmd_4,slave_tau_cmd_5,slave_tau_cmd_6,slave_tau_cmd_7";
	else if (type == 'e')
		return "slave_tau_ext_1,slave_tau_ext_2,slave_tau_ext_3,slave_tau_ext_4,slave_tau_ext_5,slave_tau_ext_6,slave_tau_ext_7";
	else if (type == 'o')
		return "slave_tau_slave_interaction_1,slave_tau_slave_interaction_2,slave_tau_slave_interaction_3,slave_tau_slave_interaction_4,slave_tau_slave_interaction_5,slave_tau_slave_interaction_6,slave_tau_slave_interaction_7";
	else if (type == 'i')
		return "slave_tau_slave_impedance_1,slave_tau_slave_impedance_2,slave_tau_slave_impedance_3,slave_tau_slave_impedance_4,slave_tau_slave_impedance_5,slave_tau_slave_impedance_6,slave_tau_slave_impedance_7";
}


string GetSymbol_JointDamping() {
	return "b_1,b_2,b_3,b_4,b_5,b_6,b_7";
}

string GetSymbol_JointDamping_Master() {
	return "master_b_1,master_b_2,master_b_3,master_b_4,master_b_5,master_b_6,master_b_7";
}

string GetSymbol_JointDamping_Slave() {
	return "slave_b_1,slave_b_2,slave_b_3,slave_b_4,slave_b_5,slave_b_6,slave_b_7";
}


string GetSymbol_JointOffset() {
	return "del_q_1,del_q_2,del_q_3,del_q_4,del_q_5,del_q_6,del_q_7";
}

string GetSymbol_JointOffset_Master() {
	return "master_del_q_1,master_del_q_2,master_del_q_3,master_del_q_4,master_del_q_5,master_del_q_6,master_del_q_7";
}

string GetSymbol_JointOffset_Slave() {
	return "slave_del_q_1,slave_del_q_2,slave_del_q_3,slave_del_q_4,slave_del_q_5,slave_del_q_6,slave_del_q_7";
}

//test_Sreekanth
string getCurrentTorque() {
	return "currentTorque";
}