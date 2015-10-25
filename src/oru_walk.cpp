/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

//#include <qi/os.hpp>

#include "oru_walk.h"


/**
 * Constructor for oru_walk object
 * @param broker The parent broker
 * @param name The name of the module
 */
//oru_walk::oru_walk(const string& name) : wp (broker)
oru_walk::oru_walk(const string& name, std::string connection_ip, int connection_port):
    connection_ip_(connection_ip), connection_port_(connection_port),dcm_loop_break(false)
{
    solver = NULL;
}



/**
 * Destructor for oru_walk object
 */
oru_walk::~oru_walk()
{
    setStiffness(0.0f);
    // Remove the postProcess call back connection
    //printMessage ("Module destroyed.\n");
    if (solver != NULL)
    {
        delete solver;
        solver = NULL;
    }
}

/**
 * @brief 
 */
void oru_walk::initRobot()
{
    robot_interface_.reset(new oru_robot_interface(connection_ip_.c_str(), connection_port_, joint_names));

    int index[LOWER_JOINTS_NUM];
    std::vector<double> init_joint_positions;
    std::vector<double> init_joint_velocities;

    init_joint_positions.resize(LOWER_JOINTS_NUM);
    init_joint_velocities.resize(LOWER_JOINTS_NUM);

    for (int i=LOWER_JOINTS_NUM; i< JOINTS_NUM; i++) {
        index[i-LOWER_JOINTS_NUM] = i;
        init_joint_positions[i-LOWER_JOINTS_NUM] = joint_commands[i];
        init_joint_velocities[i-LOWER_JOINTS_NUM] = 0.1;
    }

    //robot_interface_->sendVelocityCommand(index, init_joint_velocities, LOWER_JOINTS_NUM);
    //sleep(1.0);

    robot_interface_->sendPositionsCommand(index, init_joint_positions, LOWER_JOINTS_NUM);
    sleep(2);

    for (int i=0; i< LOWER_JOINTS_NUM; i++) {
        index[i] = i;
        init_joint_positions[i] = joint_commands[i];
    }

    robot_interface_->sendVelocityCommand(index, init_joint_velocities, LOWER_JOINTS_NUM);
    //sleep(1.0);
    robot_interface_->sendPositionsCommand(index, init_joint_positions, LOWER_JOINTS_NUM);
    sleep(2.0);

    printMessage("Robot is initilized.\n");
}
