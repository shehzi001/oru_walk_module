/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */


#ifndef ORU_ROBOT_INTERFACE_H
#define ORU_ROBOT_INTERFACE_H


//----------------------------------------
// INCLUDES
//----------------------------------------
#include "extApi.h"

#include <math.h>
#include "vrep/VRepJointInterface.h"
#include <time.h>
#include <boost/bind.hpp> // callback hook
#include <boost/thread.hpp>
#include <vector>
#include <iostream>

#define _USE_MATH_DEFINES

#define ALPHA 0.8
//----------------------------------------
// DEFINITIONS
//----------------------------------------
using namespace std;


/**
 * @brief The main walking module class.
 */
class oru_robot_interface
{
    public:
        /**
         * Ctor.
         */
        oru_robot_interface(const char* connection_ip, int connection_port, std::vector<string> joint_names);

        /**
         * Dtor.
         */
        ~oru_robot_interface ();

        /**
         * Sends position commands to joint position controller.
         */
        void sendPositionsCommand(int index[], std::vector<double> positions, int size);

        /**
         * Sends velocity commands to joint position controller.
         */
        void sendVelocityCommand(int index[], std::vector<double> velocities, int size);

        /**
         * Reads current joint positions.
         */
        void readJointPositions(int index[],  int size, std::vector<double> &positions);

        void getJointVelocity();

        void getIMUData(std::vector<double> &imu_data);

        void getTorsoFeedback(std::vector<double> &torso_feedback);

    private:
        /**
         * Copy Ctor.
         */
        oru_robot_interface(const oru_robot_interface &other);

        /**
         * Assignment operator
         */
        oru_robot_interface &operator=(const oru_robot_interface &other);

    public:
        std::vector<string> joint_names;
        std::vector<double> current_joint_values;
        std::vector<string> imu_signal_names;
        int no_of_imu_signals;
        std::vector<double> imu_data_buffer;
        clock_t pT;
        double pitch_g, pitch_a, pitch_imu;
        double roll_g, roll_a, roll_imu;

    private:
        boost::shared_ptr<VRepJointInterface> vrep_joint_interface_;
};

#endif  // ORU_ROBOT_INTERFACE_H
