/**
 * @file
 * @author Alexander Sherikov
 */

#include "walk_parameters.h"
#include "smpc_solver.h"
#include <iostream>



/**
 * @brief Initialize parameters to default values.
 *
 * @param[in] broker parent broker.
 */
//walkParameters::walkParameters(ALPtr<ALBroker> broker) :
//    pref_proxy(broker)
walkParameters::walkParameters()
{
}

void walkParameters::walkParametersInitialize()
{
    // CoM position feedback
    /**
     * @ref AldNaoPaper "0.5 in the publication by Aldebaran-Robotics"
     */
    feedback_gain = 0.3;
    /**
     * @ref AldNaoPaper "0.003 in the publication by Aldebaran-Robotics"
     * Note, that even though they derive the value of this parameter in 
     * the paper, we simply tuned it.
     */
    feedback_threshold = 0.004;


// parameters of the MPC solver
    mpc_solver_type = SOLVER_TYPE_AS;

    mpc_gain_position = 8000.0;   // closeness to the reference ZMP points 
    mpc_gain_acceleration = 0.02; // penalty for the acceleration
    mpc_gain_velocity = 1.0;      // penalty for the velocity
    mpc_gain_jerk = 1.0;          // penalty for the jerk

    mpc_as_tolerance = 1e-7;
    mpc_as_max_activate = 20;
    mpc_as_use_downdate = true;
    
    mpc_ip_tolerance_int = 1e-1;
    mpc_ip_tolerance_ext = 1e+4;
    mpc_ip_t = 1e-1;
    mpc_ip_mu = 1.0;
    mpc_ip_bs_alpha = 0.01;
    mpc_ip_bs_beta = 0.9;
    mpc_ip_max_iter = 5;
    mpc_ip_bs_type = smpc::SMPC_IP_BS_LOGBAR;


// parameters of the walking pattern generator
    preview_window_size = 40;
    preview_sampling_time_ms = 40;
    preview_sampling_time_sec = (double) preview_sampling_time_ms / 1000;

    ss_time_ms = 400;
    ds_time_ms = 40;
    ds_number = 7;
    step_pairs_number = 8;


// parameters of the steps
    /**
     * @ref AldNaoPaper "0.015 in the publication by Aldebaran-Robotics"
     * 0.02 is used in the built-in module, but is is slightly higher when 
     * executed on the robot.
     */
    step_height = 0.02;

    // The longest step in the built-in module.
    step_length = 0.04;

    // Refer to smpc_solver/include/WMG.h for the description of these  parameters.
    bezier_weight_1 = 1.5;
    bezier_weight_2 = 3.0;
    bezier_inclination_1 = 0.015;
    bezier_inclination_2 = 0.01;

    // assume that z coordinate of a foot after position correction is 0.0
    set_support_z_to_zero = true;


// parameters of the control thread    
    /*
     * Accordingly to the NAOqi documentation
     * @verbatim
        Only DCM has a real-time thread in NAOqi. ::
            scheduling policy: SCHED_FIFO
            scheduling priority: 90
       @endverbatim
     * Cannot set priority >= 70.
     */
    walk_control_thread_priority = 65; // constant

    dcm_time_shift_ms = 0;
    dcm_sampling_time_ms = 10; // constant

    control_sampling_time_ms = 20; // constant
    control_sampling_time_sec = (double) control_sampling_time_ms / 1000;

    loop_time_limit_ms = 15; // less than control_sampling_time_ms


    walk_pattern = WALK_PATTERN_STRAIGHT;


// IGM    
    // Acc. to the reference: "32 x Hall effect sensors. 12 bit precision, ie 4096 values per turn"
    // i.e. 3.14*2/4096 = 0.0015 radian is the lowest detectable change in a joint angle.
    igm_tol = 0.0015;
    igm_max_iter = 20;
    igm_mu = 1.0;



// list of parameters, that are stored in config file:

    //param_names.arraySetSize(NUM_PARAMETERS);

    param_names[FEEDBACK_GAIN]            = "feedback_gain";
    param_names[FEEDBACK_THRESHOLD]       = "feedback_threshold";

    param_names[MPC_SOLVER_TYPE]          = "mpc_solver_type";

    param_names[MPC_GAIN_VELOCITY]        = "mpc_gain_velocity";
    param_names[MPC_GAIN_POSITION]        = "mpc_gain_position";
    param_names[MPC_GAIN_JERK]            = "mpc_gain_jerk";
    param_names[MPC_GAIN_ACCELERATION]    = "mpc_gain_acceleration";

    param_names[MPC_AS_TOLERANCE]         = "mpc_as_tolerance";
    param_names[MPC_AS_MAX_ACTIVATE]      = "mpc_as_max_activate";
    param_names[MPC_AS_USE_DOWNDATE]      = "mpc_as_use_downdate";
                                             
    param_names[MPC_IP_TOLERANCE_INT]     = "mpc_ip_tolerance_int";
    param_names[MPC_IP_TOLERANCE_EXT]     = "mpc_ip_tolerance_ext";
    param_names[MPC_IP_T]                 = "mpc_ip_t";
    param_names[MPC_IP_MU]                = "mpc_ip_mu";
    param_names[MPC_IP_BS_ALPHA]          = "mpc_ip_bs_alpha";
    param_names[MPC_IP_BS_BETA]           = "mpc_ip_bs_beta";
    param_names[MPC_IP_MAX_ITER]          = "mpc_ip_max_iter";
    param_names[MPC_IP_BS_TYPE]           = "mpc_ip_bs_type";
                                            
    param_names[IGM_MU]                   = "igm_mu";

    param_names[STEP_HEIGHT]              = "step_height";
    param_names[STEP_LENGTH]              = "step_length";
    param_names[BEZIER_WEIGHT_1]          = "bezier_weight_1";
    param_names[BEZIER_WEIGHT_2]          = "bezier_weight_2";
    param_names[BEZIER_INCLINATION_1]     = "bezier_inclination_1";
    param_names[BEZIER_INCLINATION_2]     = "bezier_inclination_2";

    param_names[LOOP_TIME_LIMIT_MS]       = "loop_time_limit_ms";
    param_names[DCM_TIME_SHIFT_MS]        = "dcm_time_shift_ms";
    param_names[PREVIEW_SAMPLING_TIME_MS] = "preview_sampling_time_ms";
    param_names[PREVIEW_WINDOW_SIZE]      = "preview_window_size";     

    param_names[SS_CONTROL_LOOPS]         = "ss_control_loops";     
    param_names[DS_CONTROL_LOOPS]         = "ds_control_loops";     
    param_names[DS_NUMBER]                = "ds_number";     
    param_names[STEP_PAIRS_NUMBER]        = "step_pairs_number";     

    param_names[WALK_PATTERN]             = "walk_pattern";
}



/**
 * @brief Read parameters from configuration file; if the file does
 *  not exist, write the default values to it.
 */
void walkParameters::readParameters()
{
    /*
    ALValue preferences;

    try
    {
        preferences = pref_proxy.readPrefFile ("oru_walk", false);
    }
    catch (const ALError &e)
    {
        qiLogInfo ("module.oru_walk") << e.what();
        writeParameters ();
        return;
    }


    if (!preferences.isArray())
    {
        return;
    }

    for (int i = 0; i < preferences.getSize(); i++)
    {
        if (preferences[i][2].isFloat())
        {
            if(preferences[i][0] == param_names[FEEDBACK_GAIN])         { feedback_gain        = preferences[i][2]; }
            if(preferences[i][0] == param_names[FEEDBACK_THRESHOLD])    { feedback_threshold   = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_GAIN_POSITION])     { mpc_gain_position    = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_GAIN_VELOCITY])     { mpc_gain_velocity    = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_GAIN_ACCELERATION]) { mpc_gain_acceleration= preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_GAIN_JERK])         { mpc_gain_jerk        = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_AS_TOLERANCE])      { mpc_as_tolerance     = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_TOLERANCE_INT])  { mpc_ip_tolerance_int = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_TOLERANCE_EXT])  { mpc_ip_tolerance_ext = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_T])              { mpc_ip_t             = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_MU])             { mpc_ip_mu            = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_BS_ALPHA])       { mpc_ip_bs_alpha      = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_BS_BETA])        { mpc_ip_bs_beta       = preferences[i][2]; }
            if(preferences[i][0] == param_names[IGM_MU])                { igm_mu               = preferences[i][2]; }
            if(preferences[i][0] == param_names[STEP_HEIGHT])           { step_height          = preferences[i][2]; }
            if(preferences[i][0] == param_names[STEP_LENGTH])           { step_length          = preferences[i][2]; }
            if(preferences[i][0] == param_names[BEZIER_WEIGHT_1])       { bezier_weight_1      = preferences[i][2]; }
            if(preferences[i][0] == param_names[BEZIER_WEIGHT_2])       { bezier_weight_2      = preferences[i][2]; }
            if(preferences[i][0] == param_names[BEZIER_INCLINATION_1])  { bezier_inclination_1 = preferences[i][2]; }
            if(preferences[i][0] == param_names[BEZIER_INCLINATION_2])  { bezier_inclination_2 = preferences[i][2]; }
        }
        if (preferences[i][2].isInt())
        {
            if(preferences[i][0] == param_names[MPC_SOLVER_TYPE])     { mpc_solver_type     = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_MAX_ITER])     { mpc_ip_max_iter     = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_IP_BS_TYPE])      { mpc_ip_bs_type      = preferences[i][2]; }
            if(preferences[i][0] == param_names[MPC_AS_MAX_ACTIVATE]) { mpc_as_max_activate = preferences[i][2]; }

            if(preferences[i][0] == param_names[LOOP_TIME_LIMIT_MS])  { loop_time_limit_ms  = preferences[i][2]; }
            if(preferences[i][0] == param_names[DCM_TIME_SHIFT_MS])   { dcm_time_shift_ms   = preferences[i][2]; }
            if(preferences[i][0] == param_names[PREVIEW_WINDOW_SIZE]) { preview_window_size = preferences[i][2]; }
            if(preferences[i][0] == param_names[DS_NUMBER])           { ds_number           = preferences[i][2]; }     
            if(preferences[i][0] == param_names[STEP_PAIRS_NUMBER])   { step_pairs_number   = preferences[i][2]; }
            if(preferences[i][0] == param_names[WALK_PATTERN])        { walk_pattern        = preferences[i][2]; }


            if(preferences[i][0] == param_names[PREVIEW_SAMPLING_TIME_MS])
            {
                preview_sampling_time_ms = preferences[i][2];
                control_sampling_time_sec = (double) control_sampling_time_ms / 1000;
            }

            if (preferences[i][0] == param_names[SS_CONTROL_LOOPS]) 
                {ss_time_ms = control_sampling_time_ms * (int) preferences[i][2];}

            if(preferences[i][0] == param_names[DS_CONTROL_LOOPS])
                {ds_time_ms = control_sampling_time_ms * (int) preferences[i][2];}
        }
        if (preferences[i][2].isBool())
        {
            if(preferences[i][0] == param_names[MPC_AS_USE_DOWNDATE]) { mpc_as_use_downdate = preferences[i][2]; }
        }
    }
    */
}



/**
 * @brief Write the values of parameters to the config file.
 */
void walkParameters::writeParameters()
{
    /*
    ALValue preferences;

    preferences.arraySetSize(NUM_PARAMETERS);
    for (int i = 0; i < NUM_PARAMETERS; i++)
    {
        preferences[i].arraySetSize(3);
        preferences[i][0] = param_names[i];
    }


    // descriptions
    preferences[FEEDBACK_GAIN][1]            = "";
    preferences[FEEDBACK_THRESHOLD][1]       = "";

    preferences[MPC_SOLVER_TYPE][1]          = "";

    preferences[MPC_GAIN_POSITION][1]        = "";
    preferences[MPC_GAIN_VELOCITY][1]        = "";
    preferences[MPC_GAIN_ACCELERATION][1]    = "";
    preferences[MPC_GAIN_JERK][1]            = "";
    
    preferences[MPC_AS_TOLERANCE][1]         = "";
    preferences[MPC_AS_MAX_ACTIVATE][1]      = "";
    preferences[MPC_AS_USE_DOWNDATE][1]      = "";
                    
    preferences[MPC_IP_TOLERANCE_INT][1]     = "";
    preferences[MPC_IP_TOLERANCE_EXT][1]     = "";
    preferences[MPC_IP_T][1]                 = "";
    preferences[MPC_IP_MU][1]                = "";
    preferences[MPC_IP_BS_ALPHA][1]          = "";
    preferences[MPC_IP_BS_BETA][1]           = "";
    preferences[MPC_IP_MAX_ITER][1]          = "";
    preferences[MPC_IP_BS_TYPE][1]           = "";

    preferences[IGM_MU][1]                   = "";

    preferences[STEP_HEIGHT][1]              = "";
    preferences[STEP_LENGTH][1]              = "";
    preferences[BEZIER_WEIGHT_1][1]          = "";
    preferences[BEZIER_WEIGHT_2][1]          = "";
    preferences[BEZIER_INCLINATION_1][1]     = "";
    preferences[BEZIER_INCLINATION_2][1]     = "";

    preferences[LOOP_TIME_LIMIT_MS][1]       = "";
    preferences[DCM_TIME_SHIFT_MS][1]        = "";
    preferences[PREVIEW_SAMPLING_TIME_MS][1] = "";
    preferences[PREVIEW_WINDOW_SIZE][1]      = "";     

    preferences[SS_CONTROL_LOOPS][1]         = "";
    preferences[DS_CONTROL_LOOPS][1]         = "";
    preferences[DS_NUMBER][1]                = "";     
    preferences[STEP_PAIRS_NUMBER][1]        = "";     

    preferences[WALK_PATTERN][1]             = "";     


    // values
    preferences[FEEDBACK_GAIN][2]            = feedback_gain;
    preferences[FEEDBACK_THRESHOLD][2]       = feedback_threshold;

    preferences[MPC_SOLVER_TYPE][2]          = mpc_solver_type;

    preferences[MPC_GAIN_POSITION][2]        = mpc_gain_position;
    preferences[MPC_GAIN_VELOCITY][2]        = mpc_gain_velocity;
    preferences[MPC_GAIN_ACCELERATION][2]    = mpc_gain_acceleration;
    preferences[MPC_GAIN_JERK][2]            = mpc_gain_jerk;

    preferences[MPC_AS_TOLERANCE][2]         = mpc_as_tolerance;
    preferences[MPC_AS_MAX_ACTIVATE][2]      = mpc_as_max_activate;
    preferences[MPC_AS_USE_DOWNDATE][2]      = mpc_as_use_downdate;

    preferences[MPC_IP_TOLERANCE_INT][2]     = mpc_ip_tolerance_int;
    preferences[MPC_IP_TOLERANCE_EXT][2]     = mpc_ip_tolerance_ext;
    preferences[MPC_IP_T][2]                 = mpc_ip_t;
    preferences[MPC_IP_MU][2]                = mpc_ip_mu;
    preferences[MPC_IP_BS_ALPHA][2]          = mpc_ip_bs_alpha;
    preferences[MPC_IP_BS_BETA][2]           = mpc_ip_bs_beta;
    preferences[MPC_IP_MAX_ITER][2]          = mpc_ip_max_iter;
    preferences[MPC_IP_BS_TYPE][2]           = mpc_ip_bs_type;

    preferences[IGM_MU][2]                   = igm_mu;

    preferences[STEP_HEIGHT][2]              = step_height;
    preferences[STEP_LENGTH][2]              = step_length;
    preferences[BEZIER_WEIGHT_1][2]          = bezier_weight_1;
    preferences[BEZIER_WEIGHT_2][2]          = bezier_weight_2;
    preferences[BEZIER_INCLINATION_1][2]     = bezier_inclination_1;
    preferences[BEZIER_INCLINATION_2][2]     = bezier_inclination_2;

    preferences[LOOP_TIME_LIMIT_MS][2]       = loop_time_limit_ms;
    preferences[DCM_TIME_SHIFT_MS][2]        = dcm_time_shift_ms;
    preferences[PREVIEW_SAMPLING_TIME_MS][2] = preview_sampling_time_ms;
    preferences[PREVIEW_WINDOW_SIZE][2]      = preview_window_size;

    preferences[SS_CONTROL_LOOPS][2]         = ss_time_ms / control_sampling_time_ms;
    preferences[DS_CONTROL_LOOPS][2]         = ds_time_ms / control_sampling_time_ms;
    preferences[DS_NUMBER][2]                = ds_number;
    preferences[STEP_PAIRS_NUMBER][2]        = step_pairs_number;

    preferences[WALK_PATTERN][2]             = walk_pattern;

    try
    {
        pref_proxy.writePrefFile("oru_walk", preferences, true); 
    }
    catch (const ALError &e)
    {
        qiLogInfo ("module.oru_walk") << e.what();
    }
    */
}
