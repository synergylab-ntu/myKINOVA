
using namespace std::chrono;
// Includes for unix_epoch - start
#include <time.h>
#include <ctime>
#include <chrono>
// Includes for unix_epoch - end

#include <Shlwapi.h>
#pragma comment(lib, "shlwapi.lib")


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


class myKINOVA_LOG
{
public:
    // newvars
    // data management variables
    int i;
    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    
    int logging = 1;      //change to zero to manually start logging
    int focus = 0;
    int data_count_final;
    // newvars

    int                             data_count = 0;
    int                             ACTUATOR_COUNT = 7;
    int64_t                         timestamp;
    


    int                     DURATION;
    int arr_length;
    double* time_log;
    int64_t* unix_epoch;
    float** UDP_q_log;
    float** UDP_tau_log;
    float** ext_tau_log;
    float** tau_cmd_log;
    k_api::BaseCyclic::Feedback* data_log;

    int64_t GetTickUs()
    {
#if defined(_MSC_VER)
        LARGE_INTEGER start, frequency;

        QueryPerformanceFrequency(&frequency);
        QueryPerformanceCounter(&start);

        timestamp = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
        now = (start.QuadPart * 1000000) / frequency.QuadPart;
        return now;
#else
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);

        return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
    }


    void write2LOG(int data_count_in, k_api::BaseCyclic::Feedback base_feedback, float ext_tau[7], float UDP_q[7], float UDP_tau[7], float tau_cmd[7])
    {
        data_count = data_count_in;

        // Data logging starts

        time_log[data_count] = GetTickUs();

        unix_epoch[data_count] = timestamp; // Make sure GetTickUs() is called before timestamp is recorded into unix_epoch.

        // Didn't work - figure out the pointers                                   
        //std::cout << access_my_nD(UDP_f_log, 0, 0) << std::endl;

        // data_log logging
        data_log[data_count] = base_feedback;

        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            ext_tau_log[data_count][i] = ext_tau[i];
            tau_cmd_log[data_count][i] = tau_cmd[i];
            UDP_q_log[data_count][i] = UDP_q[i];
            UDP_tau_log[data_count][i] = UDP_tau[i];
        }

    }


    // Some how the following tricks didn't work in initialising the nD arrays using functions

    void define_nD_array(float** pointer, int arr_length, int my_dim) {
        pointer = new float* [arr_length];
        for (int i = 0; i < arr_length; i++)
            pointer[i] = new float[my_dim];

    }

    float access_my_nD(float** pointer, int i, int j) {
        return pointer[i][j];
    }

    myKINOVA_LOG() {// default constructor

    }

    myKINOVA_LOG(int duration_in) {
        now = GetTickUs();
        DURATION = duration_in;
        arr_length = DURATION * 1000;
        time_log = new double[arr_length];
        unix_epoch = new int64_t[arr_length];

        data_log = new k_api::BaseCyclic::Feedback[arr_length];

        // defining the multi dimensional arrays
        //define_nD_array(UDP_f_log,arr_length, dim_f);
        UDP_q_log = new float* [arr_length];
        for (int i = 0; i < arr_length; i++)
            UDP_q_log[i] = new float[ACTUATOR_COUNT];

        UDP_tau_log = new float* [arr_length];
        for (int i = 0; i < arr_length; i++)
            UDP_tau_log[i] = new float[ACTUATOR_COUNT];

        ext_tau_log = new float* [arr_length];
        for (int i = 0; i < arr_length; i++)
            ext_tau_log[i] = new float[ACTUATOR_COUNT];

        tau_cmd_log = new float* [arr_length];
        for (int i = 0; i < arr_length; i++)
            tau_cmd_log[i] = new float[ACTUATOR_COUNT];
    }
};

class myWRITE_KINOVA_LOG {

public:
    int duration;
    std::string exp_folder;
    char EXEPATH[MAX_PATH];
    std::string ROBOT_IP;
    int ACTUATOR_COUNT = 7;
    int data_count_final;

    std::string GetTimestamp(time_t now) {
        tm* ltm = localtime(&now);
        std::string year = std::to_string(1900 + ltm->tm_year);
        std::string month = std::to_string(1 + ltm->tm_mon);
        std::string day = std::to_string(ltm->tm_mday);
        std::string hour = std::to_string(ltm->tm_hour);
        std::string min = std::to_string(ltm->tm_min);
        std::string sec = std::to_string(ltm->tm_sec);
        std::string timestamp = year + "_" + month + "_" + day + "_" + hour + min + sec + ".csv";
        return timestamp;
    }

    TCHAR* GetEXEpath()
    {
        char buffer[MAX_PATH];
        GetModuleFileName(NULL, buffer, MAX_PATH);
        std::cout << "Executable path: " << buffer << std::endl;
        return buffer;
    }

    // Custom string functions
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

    void get_exp_folder_v2()
    {
        GetModuleFileName(NULL, EXEPATH, MAX_PATH);
        //std::cout << EXEPATH << std::endl;
        DWORD length = GetModuleFileName(NULL, EXEPATH, MAX_PATH);

        wchar_t wtext[MAX_PATH];
        mbstowcs(wtext, EXEPATH, strlen(EXEPATH) + 1);//Plus null
        LPWSTR ptr = wtext;

        std::string s1(EXEPATH);
        //cout << s1.substr(0, s1.find_last_of("\\/")) << endl;

        std::string s2 = s1.substr(0, s1.find_last_of("\\/"));

        //std::cout << s2 << std::endl;

        exp_folder = s2;

        std::cout << "Executable path: " << exp_folder << std::endl;

        replace_all(exp_folder, "\\", "/");
        std::cout << "Executable path: " << exp_folder << std::endl;

        std::string exp_settings = exp_folder;
        exp_settings.append("/exp_settings.txt");
        std::cout << "exp settings file is at: " << exp_settings << std::endl;

        std::string read_exp_line;
        std::ifstream MyReadFile(exp_settings);
        std::getline(MyReadFile, read_exp_line);
        replace_all(read_exp_line, "\\", "/");
        read_exp_line.erase(0, 1);
        exp_folder.append("/");
        exp_folder.append(read_exp_line);
        exp_folder.append("/");
        std::cout << exp_folder << std::endl;

    }

    void write2FILE(myKINOVA_LOG DATA, std::shared_ptr<rl::mdl::Model> model)
        {
        rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());
        rl::math::Vector q(dynamics->getDof());
        rl::math::Vector qd(dynamics->getDof());
        rl::math::Vector qdd(dynamics->getDof());
        q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        qdd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        time_t now_t = time(0);
        std::string file_name = "kinova_log_file_";

        std::cout << "Writing to file..." << std::endl;
        //
        get_exp_folder_v2();
        std::string base_path = exp_folder;
        //
        std::string timestamp = GetTimestamp(now_t);
        //
        std::ofstream log_file(base_path + file_name + "_" + ROBOT_IP + "_" + timestamp);

        std::cout << "Writing to log now" << std::endl;
        if (log_file.is_open())
        {
            log_file << "Robot IP:" << ROBOT_IP << "\n";

            log_file << "Time(ms),Time(s),Unix_epoch(ns),"
                << GetSymbol_JointAngles('f') << ","
                << GetSymbol_JointVelocity('f') << ","
                << GetSymbol_JointTorque('f') << ","
                << GetSymbol_JointTorque('g') << ","
                << GetSymbol_JointTorque('v') << ","
                << GetSymbol_JointTorque('m') << ","
                << GetSymbol_JointTorque('c') << ","

                << "V_1,V_2,V_3,V_4,V_5,V_6,V_7" << ","
                << "I_1,I_2,I_3,I_4,I_5,I_6,I_7" << ","
                << "acc_x,acc_y,acc_z,"
                << "omega_x, omega_y, omega_z,"
                << "ARM_I,ARM_V,"
                << "acc_base_x, acc_base_y, acc_base_z,"
                << "omega_base_x, omega_base_y, omega_base_z,"
                << "tool_pose_x, tool_pose_y, tool_pose_z,"
                << "tool_pose_theta_x, tool_pose_theta_y, tool_pose_theta_z,"
                << "tool_twist_linear_x, tool_twist_linear_y, tool_twist_linear_z,"
                << "tool_twist_angular_x, tool_twist_angular_x, tool_twist_angular_x,"
                << "tool_external_wrench_force_x, tool_external_wrench_force_y, tool_external_wrench_force_z,"
                << "tool_external_wrench_torque_x, tool_external_wrench_torque_y, tool_external_wrench_torque_z" << ", "
                << "UDP_q1,UDP_q2,UDP_q3,UDP_q4,UDP_q5,UDP_q6,UDP_q7" << ","
                << "UDP_tau1,UDP_tau2,UDP_tau3,UDP_tau4,UDP_tau5,UDP_tau6,UDP_tau7" << ","
                << "ext_tau1,ext_tau2,ext_tau3,ext_tau4,ext_tau5,ext_tau6,ext_tau7" << ","
                << "Index\n";
            std::cout << DATA.arr_length << std::endl;
            for (int i = 0; i < duration * 1000 && i < data_count_final - 1; ++i)
            {
                log_file << (DATA.time_log[i] - DATA.time_log[0]) / 1000 << ",";
                log_file << (DATA.time_log[i] - DATA.time_log[0]) / 1000000 << ",";
                log_file << DATA.unix_epoch[i] << ",";


                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.data_log[i].actuators(j).position() * rl::math::DEG2RAD << ",";
                    q[j] = (DATA.data_log[i].actuators(j).position() * rl::math::DEG2RAD);
                }

                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.data_log[i].actuators(j).velocity() * rl::math::DEG2RAD << ",";
                    qd[j] = DATA.data_log[i].actuators(j).velocity() * rl::math::DEG2RAD;
                }

                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.data_log[i].actuators(j).torque() << ",";
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
                    log_file << DATA.tau_cmd_log[i][j] << ",";
                }

                // added on 2024-02-06
                // "V_1,V_2,V_3,V_4,V_5,V_6,V_7"
                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.data_log[i].actuators(j).voltage() << ",";
                }

                // "I_1,I_2,I_3,I_4,I_5,I_6,I_7"
                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.data_log[i].actuators(j).current_motor() << ",";
                }

                // "acc_x,acc_y,acc_z,"
                log_file << DATA.data_log[i].interconnect().imu_acceleration_x() << ",";
                log_file << DATA.data_log[i].interconnect().imu_acceleration_y() << ",";
                log_file << DATA.data_log[i].interconnect().imu_acceleration_z() << ",";

                // "omega_x, omega_y, omega_z,"
                log_file << DATA.data_log[i].interconnect().imu_angular_velocity_x() << ",";
                log_file << DATA.data_log[i].interconnect().imu_angular_velocity_y() << ",";
                log_file << DATA.data_log[i].interconnect().imu_angular_velocity_z() << ",";

                // "ARM_I,ARM_V,"
                log_file << DATA.data_log[i].base().arm_current() << ",";
                log_file << DATA.data_log[i].base().arm_voltage() << ",";

                // "acc_base_x, acc_base_y, acc_base_z,"
                log_file << DATA.data_log[i].base().imu_acceleration_x() << ",";
                log_file << DATA.data_log[i].base().imu_acceleration_y() << ",";
                log_file << DATA.data_log[i].base().imu_acceleration_z() << ",";

                // "omega_base_x, omega_base_y, omega_base_z,"
                log_file << DATA.data_log[i].base().imu_angular_velocity_x() << ",";
                log_file << DATA.data_log[i].base().imu_angular_velocity_y() << ",";
                log_file << DATA.data_log[i].base().imu_angular_velocity_z() << ",";

                // "tool_pose_x, tool_pose_y, tool_pose_z,"
                log_file << DATA.data_log[i].base().tool_pose_x() << ",";
                log_file << DATA.data_log[i].base().tool_pose_y() << ",";
                log_file << DATA.data_log[i].base().tool_pose_z() << ",";

                // "tool_pose_theta_x, tool_pose_theta_y, tool_pose_theta_z,"
                log_file << DATA.data_log[i].base().tool_pose_theta_x() << ",";
                log_file << DATA.data_log[i].base().tool_pose_theta_y() << ",";
                log_file << DATA.data_log[i].base().tool_pose_theta_z() << ",";

                // "tool_twist_linear_x, tool_twist_linear_y, tool_twist_linear_z,"
                log_file << DATA.data_log[i].base().tool_twist_linear_x() << ",";
                log_file << DATA.data_log[i].base().tool_twist_linear_y() << ",";
                log_file << DATA.data_log[i].base().tool_twist_linear_z() << ",";

                // "tool_twist_angular_x, tool_twist_angular_x, tool_twist_angular_x,"
                log_file << DATA.data_log[i].base().tool_twist_angular_x() << ",";
                log_file << DATA.data_log[i].base().tool_twist_angular_y() << ",";
                log_file << DATA.data_log[i].base().tool_twist_angular_z() << ",";

                // "tool_external_wrench_force_x, tool_external_wrench_force_y, tool_external_wrench_force_z,"
                log_file << DATA.data_log[i].base().tool_external_wrench_force_x() << ",";
                log_file << DATA.data_log[i].base().tool_external_wrench_force_y() << ",";
                log_file << DATA.data_log[i].base().tool_external_wrench_force_z() << ",";

                // "tool_external_wrench_torque_x, tool_external_wrench_torque_y, tool_external_wrench_torque_z" << ", "
                log_file << DATA.data_log[i].base().tool_external_wrench_torque_x() << ",";
                log_file << DATA.data_log[i].base().tool_external_wrench_torque_y() << ",";
                log_file << DATA.data_log[i].base().tool_external_wrench_torque_z() << ",";

                // "UDP_q1,UDP_q2,UDP_q3,UDP_q4,UDP_q5,UDP_q6,UDP_q7" << ","
                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.UDP_q_log[i][j] << ",";
                }

                // "UDP_tau1,UDP_tau2,UDP_tau3,UDP_tau4,UDP_tau5,UDP_tau6,UDP_tau7" << ","
                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.UDP_tau_log[i][j] << ",";
                }

                // "ext_tau1,ext_tau2,ext_tau3,ext_tau4,ext_tau5,ext_tau6,ext_tau7"
                for (int j = 0; j < ACTUATOR_COUNT; ++j)
                {
                    log_file << DATA.ext_tau_log[i][j] << ",";
                }

                //std::cout << "Inside the loop at iteration :" << i << std::endl;

                log_file << i << "\n";
            }
        }
        std::cout << "Writing to file completed!" << std::endl;

    }

    myWRITE_KINOVA_LOG(int duration_in, std::string ROBOT_IP_in, int data_count_final_in) {
        duration = duration_in;
        data_count_final = data_count_final_in;
        ROBOT_IP = ROBOT_IP_in;
    }
};
