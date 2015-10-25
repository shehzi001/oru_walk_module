/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"


// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
/**
 * @brief 
 */
void oru_walk::init()
{
    joint_names.resize(JOINTS_NUM);
    joint_names[HEAD_PITCH]       = "HeadPitch";
    joint_names[HEAD_YAW]         = "HeadYaw";

    joint_names[L_ANKLE_PITCH]    = "LAnklePitch3";
    joint_names[L_ANKLE_ROLL]     = "LAnkleRoll3";
    joint_names[L_ELBOW_ROLL]     = "LElbowRoll3";
    joint_names[L_ELBOW_YAW]      = "LElbowYaw3";
    joint_names[L_HIP_PITCH]      = "LHipPitch3";
    joint_names[L_HIP_ROLL]       = "LHipRoll3";
    joint_names[L_HIP_YAW_PITCH]  = "LHipYawPitch3";
    joint_names[L_KNEE_PITCH]     = "LKneePitch3";
    joint_names[L_SHOULDER_PITCH] = "LShoulderPitch3";
    joint_names[L_SHOULDER_ROLL]  = "LShoulderRoll3";
    joint_names[L_WRIST_YAW]      = "LWristYaw3";

    joint_names[R_ANKLE_PITCH]    = "RAnklePitch3";
    joint_names[R_ANKLE_ROLL]     = "RAnkleRoll3";
    joint_names[R_ELBOW_ROLL]     = "RElbowRoll3";
    joint_names[R_ELBOW_YAW]      = "RElbowYaw3";
    joint_names[R_HIP_PITCH]      = "RHipPitch3";
    joint_names[R_HIP_ROLL]       = "RHipRoll3";
    joint_names[R_HIP_YAW_PITCH]  = "RHipYawPitch3";
    joint_names[R_KNEE_PITCH]     = "RKneePitch3";
    joint_names[R_SHOULDER_PITCH] = "RShoulderPitch3";
    joint_names[R_SHOULDER_ROLL]  = "RShoulderRoll3";
    joint_names[R_WRIST_YAW]      = "RWristYaw3";

    for (int i=0; i< JOINTS_NUM; i++) {
        joint_indices_[i] = i;
    }

    initJointAngles();

    initWalkCommands();

    std::cout << "module.oru_walk: Execution of init() is finished." << std::endl;
}


void oru_walk::initJointAngles()
{
    init_joint_angles.resize(JOINTS_NUM);

    init_joint_angles[L_HIP_YAW_PITCH]  =  0.0;
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    init_joint_angles[R_HIP_YAW_PITCH]  =  0.0;

    init_joint_angles[L_HIP_ROLL]       = -0.000384;
    init_joint_angles[L_HIP_PITCH]      = -0.598291/4;//−0.14957275
    init_joint_angles[L_KNEE_PITCH]     =  1.009413/1.25;//0.8075304
    init_joint_angles[L_ANKLE_PITCH]    = -0.492352;
    init_joint_angles[L_ANKLE_ROLL]     =  0.000469;

    init_joint_angles[R_HIP_ROLL]       = -0.000384;
    init_joint_angles[R_HIP_PITCH]      = -0.598219/4;
    init_joint_angles[R_KNEE_PITCH]     =  1.009237/1.25;
    init_joint_angles[R_ANKLE_PITCH]    = -0.492248;
    init_joint_angles[R_ANKLE_ROLL]     =  0.000469;

    init_joint_angles[L_SHOULDER_PITCH] =  1.418908;
    init_joint_angles[L_SHOULDER_ROLL]  =  0.332836;
    init_joint_angles[L_ELBOW_YAW]      = -1.379108;
    init_joint_angles[L_ELBOW_ROLL]     = -1.021602;
    init_joint_angles[L_WRIST_YAW]      = -0.013848;

    init_joint_angles[R_SHOULDER_PITCH] =  1.425128;
    init_joint_angles[R_SHOULDER_ROLL]  = -0.331386;
    init_joint_angles[R_ELBOW_YAW]      =  1.383626;
    init_joint_angles[R_ELBOW_ROLL]     =  1.029356;
    init_joint_angles[R_WRIST_YAW]      = -0.01078; 

    init_joint_angles[HEAD_PITCH]       =  0.0;     
    init_joint_angles[HEAD_YAW]         =  0.0;
    /*
        init_joint_angles[L_HIP_YAW_PITCH][0]  =  0.0;
    // note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    init_joint_angles[R_HIP_YAW_PITCH][0]  =  0.0;

    init_joint_angles[L_HIP_ROLL][0]       = -0.000384;
    init_joint_angles[L_HIP_PITCH][0]      = -0.598291;
    init_joint_angles[L_KNEE_PITCH][0]     =  1.009413;
    init_joint_angles[L_ANKLE_PITCH][0]    = -0.492352;
    init_joint_angles[L_ANKLE_ROLL][0]     =  0.000469;

    init_joint_angles[R_HIP_ROLL][0]       = -0.000384;
    init_joint_angles[R_HIP_PITCH][0]      = -0.598219;
    init_joint_angles[R_KNEE_PITCH][0]     =  1.009237;
    init_joint_angles[R_ANKLE_PITCH][0]    = -0.492248;
    init_joint_angles[R_ANKLE_ROLL][0]     =  0.000469;

    init_joint_angles[L_SHOULDER_PITCH][0] =  1.418908;
    init_joint_angles[L_SHOULDER_ROLL][0]  =  0.332836;
    init_joint_angles[L_ELBOW_YAW][0]      = -1.379108;
    init_joint_angles[L_ELBOW_ROLL][0]     = -1.021602;
    init_joint_angles[L_WRIST_YAW][0]      = -0.013848;

    init_joint_angles[R_SHOULDER_PITCH][0] =  1.425128;
    init_joint_angles[R_SHOULDER_ROLL][0]  = -0.331386;
    init_joint_angles[R_ELBOW_YAW][0]      =  1.383626;
    init_joint_angles[R_ELBOW_ROLL][0]     =  1.029356;
    init_joint_angles[R_WRIST_YAW][0]      = -0.01078; 

    init_joint_angles[HEAD_PITCH][0]       =  0.0;     
    init_joint_angles[HEAD_YAW][0]         =  0.0;
    */
}

/**
 * @brief Initialize commands, that will be sent to DCM.
 */

void oru_walk::initWalkCommands()
{
    joint_commands.resize(JOINTS_NUM);
    joint_commands = init_joint_angles;

    for (int i = 0; i < LOWER_JOINTS_NUM; i++)
    {
        ref_joint_angles[i] = joint_commands[i];
    }
}


/**
 * @brief Set stiffness of joints.
 *
 * @param[in] stiffnessValue value of stiffness [0;1]
 */
void oru_walk::setStiffness(const float &stiffnessValue)
{
}