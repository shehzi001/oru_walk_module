/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "mpc_walk.h"


/**
 * @brief Set stiffness of joints.
 *
 * @param[in] stiffnessValue value of stiffness [0;1]
 */
void mpc_walk::setStiffness(const float &stiffnessValue)
{
    ALValue stiffnessCommands;


    if ((stiffnessValue < 0) || (stiffnessValue > 1))
    {
        throw ALERROR(getName(), __FUNCTION__, "Wrong parameters");
    }


    // Prepare one dcm command:
    // it will linearly "Merge" all joint stiffness
    // from last value to "stiffnessValue" in 1 seconde
    stiffnessCommands.arraySetSize(3);
    stiffnessCommands[0] = std::string("jointStiffness");
    stiffnessCommands[1] = std::string("Merge");
    stiffnessCommands[2].arraySetSize(1);
    stiffnessCommands[2][0].arraySetSize(2);
    stiffnessCommands[2][0][0] = stiffnessValue;


    /// @attention Hardcoded parameter!
    int stiffness_change_time = 1000;
    try
    {
        stiffnessCommands[2][0][1] = dcmProxy->getTime(stiffness_change_time);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error on DCM getTime : " + e.toString());
    }


    try
    {
        dcmProxy->set(stiffnessCommands);
    }
    catch (const ALError &e)
    {
        throw ALERROR (getName(), __FUNCTION__, "Error when sending stiffness to DCM : " + e.toString());
    }
}



/**
 * @brief 
 */
void mpc_walk::initPosition()
{
    ALValue initPositionCommands;

    initPositionCommands.arraySetSize(6);
    initPositionCommands[0] = string("jointActuator");
    initPositionCommands[1] = string("ClearAll");
    initPositionCommands[2] = string("time-separate");
    initPositionCommands[3] = 0;
    initPositionCommands[4].arraySetSize(1);
    initPositionCommands[5].arraySetSize(JOINTS_NUM);
    for (int i = 0; i < JOINTS_NUM; i++)
    {
        initPositionCommands[5][i].arraySetSize(1);
    }


    // --------------------------------------------------------------
    // set target values: used to define desidered joints variables: 
    // init pose with Torso z values equal to 0.3
    // --------------------------------------------------------------
    // CoM to be at 0.252
    initPositionCommands[5][L_HIP_YAW_PITCH][0]  =  0.0;        //0
    /// @todo note, that R_HIP_YAW_PITCH is controlled by the same motor as L_HIP_YAW_PITCH 
    initPositionCommands[5][R_HIP_YAW_PITCH][0]  =  0.0;        //0

    initPositionCommands[5][L_HIP_ROLL][0]       = -0.000384;   //1
    initPositionCommands[5][L_HIP_PITCH][0]      = -0.598291;   //2
    initPositionCommands[5][L_KNEE_PITCH][0]     =  1.009413;   //3
    initPositionCommands[5][L_ANKLE_PITCH][0]    = -0.492352;   //4
    initPositionCommands[5][L_ANKLE_ROLL][0]     =  0.000469;   //5

    initPositionCommands[5][R_HIP_ROLL][0]       = -0.000384;   //6
    initPositionCommands[5][R_HIP_PITCH][0]      = -0.598219;   //7
    initPositionCommands[5][R_KNEE_PITCH][0]     =  1.009237;   //8
    initPositionCommands[5][R_ANKLE_PITCH][0]    = -0.492248;   //9
    initPositionCommands[5][R_ANKLE_ROLL][0]     =  0.000469;   //10

    initPositionCommands[5][L_SHOULDER_PITCH][0] =  1.418908;   //11
    initPositionCommands[5][L_SHOULDER_ROLL][0]  =  0.332836;   //12
    initPositionCommands[5][L_ELBOW_YAW][0]      = -1.379108;   //13
    initPositionCommands[5][L_ELBOW_ROLL][0]     = -1.021602;   //14
    initPositionCommands[5][L_WRIST_YAW][0]      = -0.013848;   //15

    initPositionCommands[5][R_SHOULDER_PITCH][0] =  1.425128;   //17
    initPositionCommands[5][R_SHOULDER_ROLL][0]  = -0.331386;   //18
    initPositionCommands[5][R_ELBOW_YAW][0]      =  1.383626;   //19
    initPositionCommands[5][R_ELBOW_ROLL][0]     =  1.029356;   //20
    initPositionCommands[5][R_WRIST_YAW][0]      = -0.01078;    //21

    initPositionCommands[5][HEAD_PITCH][0]       =  0.0;        //23
    initPositionCommands[5][HEAD_YAW][0]         =  0.0;        //24
    // --------------------------------------------------------------


    // set time
    /// @attention Hardcoded parameter!
    int init_pos_time = 1200;
    try
    {
        initPositionCommands[4][0] = dcmProxy->getTime(init_pos_time);
    }
    catch (const ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error on DCM getTime : " + e.toString());
    }


    // send commands
    try
    {
        dcmProxy->setAlias(initPositionCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), __FUNCTION__, "Error with DCM setAlias : " + e.toString());
    }
}