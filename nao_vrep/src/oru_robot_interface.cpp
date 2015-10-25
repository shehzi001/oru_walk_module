
#include "oru_robot_interface.h"

/**
 * Ctor.
 */
oru_robot_interface::oru_robot_interface(const char* connection_ip, int connection_port, std::vector<string> joint_names):
        no_of_imu_signals(7), pitch_g(0), roll_g(0),pitch_a(0), roll_a(0), pitch_imu(0), roll_imu(0)
{
    vrep_joint_interface_.reset(new VRepJointInterface(connection_ip, connection_port, joint_names));

    imu_signal_names.resize(no_of_imu_signals);
    imu_data_buffer.resize(no_of_imu_signals);

    imu_signal_names[0] = "accel-X";
    imu_signal_names[1] = "accel-Y";
    imu_signal_names[2] = "accel-Z";
    imu_signal_names[3] = "gyro-X";
    imu_signal_names[4] = "gyro-Y";
    imu_signal_names[5] = "gyro-Z";
    imu_signal_names[6] = "gyro-dt";
}

/**
 * Dtor.
 */
oru_robot_interface::~oru_robot_interface ()
{

}

/**
 * Sends position commands to joint position controller.
 */
void oru_robot_interface::sendPositionsCommand(int index[], std::vector<double> positions, int size)
{
    vrep_joint_interface_->setJointPosition(index, positions, size);
}

/**
 * Sends position commands to joint position controller.
 */
void oru_robot_interface::sendVelocityCommand(int index[], std::vector<double> velocities, int size)
{
    vrep_joint_interface_->setJointVelocity(index, velocities, size);
}


/**
 * Reads current joint positions.
 */
void oru_robot_interface::readJointPositions(int index[],  int size, std::vector<double> &positions)
{
    vrep_joint_interface_->getJointPosition(index, size, positions);
}

/**
 * Reads current joint positions.
 */
void oru_robot_interface::getJointVelocity()
{
    vrep_joint_interface_->getJointVelocity(0);
}

void oru_robot_interface::getIMUData(std::vector<double> &imu_data)
{
    clock_t cT = clock();
    vrep_joint_interface_->getIMUData(imu_signal_names, imu_data);
    unsigned long dT = cT - pT;
        pT = cT;
        /*
        for(int i=0; i < 3; i++) {
            imu_data[i] = (imu_data[i] * ALPHA) + (imu_data_buffer[i]*(1.0-ALPHA));
        }
        */

        double term1 = sqrt((imu_data[1]*imu_data[1]) + (imu_data[2]*imu_data[2]));
        double term2 = sqrt((imu_data[0]*imu_data[0]) + (imu_data[2]*imu_data[2]));

        double pitch = -(atan2(imu_data[0],term1) * 180.0) / M_PI;
        double roll = (atan2(imu_data[1],term2) * 180.0) / M_PI;

        //pitch_a = (0.75*pitch_a) + (0.25*pitch);
        //roll_a = (0.75*roll_a) + (0.25*pitch);

        //pitch_g = pitch_g + imu_data[3]*(imu_data[6]);
        //roll_g = roll_g + imu_data[4]*(imu_data[6]);

        pitch_imu = (0.98)*(pitch_imu + imu_data[3] * ((double)dT/CLOCKS_PER_SEC)) + (0.02)*(pitch);
        roll_imu = (0.98)*(roll_imu + imu_data[4] * ((double)dT/CLOCKS_PER_SEC)) + (0.02)*(roll);

        //std::cout << "(roll_imu, pitch_imu) : (" << roll_imu << " , " << pitch_imu << ")" << std::endl;
}

void oru_robot_interface::getTorsoFeedback(std::vector<double> &torso_feedback)
{
    std::vector<double> imu_data;
    getIMUData(imu_data);

    torso_feedback.resize(4);
    torso_feedback[0] = pitch_imu*3.14/180;
    torso_feedback[1] = roll_imu*3.14/180;
    torso_feedback[2] = imu_data[3]*3.14/180;
    torso_feedback[3] = imu_data[4]*3.14/180;
}