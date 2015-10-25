/**
 * @file
 * @author Antonio Paolillo
 * @author Dimitar Dimitrov
 * @author Alexander Sherikov
 */

#include "oru_walk.h"
//#include "oruw_log.h"
//#include "oruw_timer.h"


/**
 * @brief Construct and initialize all necessary classes and register 
 * callback function.
 */
void oru_walk::walk()
{
    //ORUW_LOG_OPEN;

    // start walk control thread
    try
    {
        boost::thread walk_control_thread(&oru_walk::walkControl, this);
        struct sched_param walk_control_thread_sched;

        walk_control_thread_sched.sched_priority = wp.walk_control_thread_priority;
        int retval = pthread_setschedparam(
                walk_control_thread.native_handle(), 
                SCHED_FIFO, 
                &walk_control_thread_sched);
        if (retval != 0)
        {
            // Assume that this error is not critical
            //ORUW_LOG_MESSAGE("Cannot change the priority of the walk control thread: %s\n", strerror(retval));
        }

        walk_control_thread.detach();
    }
    catch (...)
    {
        halt("Failed to spawn the walk control thread.\n", __FUNCTION__);
    }

    // register callback
    dcm_loop_counter = 0;
}

/**
 * @brief Update joint angles.
 */
void oru_walk::readSensors(jointState& joint_state)
{
    vector<double> sensorValues;

    robot_interface_->readJointPositions(joint_indices_, JOINTS_NUM, sensorValues);

    for (int i = 0; i < JOINTS_NUM; i++)
    {
        joint_state.q[i] = sensorValues[i];
        if (i < LOWER_JOINTS_NUM) {
            ref_joint_angles[i] = sensorValues[i];
            //std::cout << i << ":" << ref_joint_angles[i] << " ";
        }
    }

    //std::cout << std::endl;

    /* Acc. to the documentation:
     * "LHipYawPitch and RHipYawPitch share the same motor so they move
     *  simultaneously and symmetrically. In case of conflicting orders, 
     *  LHipYawPitch always takes the priority."
     * Make sure that these joint angles are equal:
     */
    joint_state.q[R_HIP_YAW_PITCH] = joint_state.q[L_HIP_YAW_PITCH];

    //robot_interface_->getTorsoFeedback(torso_feedback_);
}



/**
 * @brief Wake up walk control thread periodically.
 */
 /**
void oru_walk::dcmCallback()
{
    // read from sensor and update joint values
    dcm_loop_counter++;
    if (dcm_loop_counter % (wp.control_sampling_time_ms / wp.dcm_sampling_time_ms) == 0)
    {
        //printMessage ("dcm callback satisfied.");
        //last_dcm_time_ms = *last_dcm_time_ms_ptr + wp.dcm_time_shift_ms;

        //printMessage ("unlocking thread.");
        boost::mutex::scoped_lock lock(walk_control_mutex);
        walk_control_condition.notify_one();
        lock.unlock();
        //printMessage ("thread unlocked.");
    }
}
*/
bool oru_walk::dcmCallback()
{

    readSensors (nao.state_sensor);
    //robot_interface_->getJointVelocity();
    boost::mutex::scoped_lock lock(walk_control_mutex);
    walk_control_condition.notify_one();
    lock.unlock();

    return dcm_loop_break;
}


/**
 * @brief The position of the next support foot may change from the targeted, 
 * this function moves it to the right place.
 */
void oru_walk::correctNextSupportPosition(WMG &wmg)
{
    Transform<double,3> swing_foot_posture;
    nao.getSwingFootPosture (nao.state_sensor, swing_foot_posture.data());
    wmg.changeNextSSPosition (swing_foot_posture.data(), wp.set_support_z_to_zero);
}


/**
 * @brief Initialize solver
 */
void oru_walk::initSolver()
{
    /*
    std::cout<< "initsolver.....feedback_gain:" << wp.feedback_gain << std::endl;
    std::cout<< "initsolver.....feedback_threshold......................:" << wp.feedback_threshold << std::endl;
    std::cout<< "initsolver.....mpc_solver_type....................:" << wp.mpc_solver_type << std::endl;
    std::cout<< "initsolver......mpc_gain_position.....................:" << wp.mpc_gain_position << std::endl;
    std::cout<< "initsolver....mpc_gain_acceleration.................:" << wp.mpc_gain_acceleration << std::endl;
    std::cout<< "initsolver....mpc_as_max_activate..................:" << wp.mpc_as_max_activate << std::endl;
    */
    if (solver != NULL)
    {
        delete solver;
        solver = NULL;
    }
    if (wp.mpc_solver_type == SOLVER_TYPE_AS)
    {
        solver = new smpc::solver_as (
                wp.preview_window_size,
                wp.mpc_gain_position,
                wp.mpc_gain_velocity,
                wp.mpc_gain_acceleration,
                wp.mpc_gain_jerk,
                wp.mpc_as_tolerance,
                wp.mpc_as_max_activate,
                wp.mpc_as_use_downdate,
                false); // objective
    }
    else if (wp.mpc_solver_type == SOLVER_TYPE_IP)
    {
        solver = new smpc::solver_ip (
                wp.preview_window_size,
                wp.mpc_gain_position,
                wp.mpc_gain_velocity,
                wp.mpc_gain_acceleration,
                wp.mpc_gain_jerk,
                wp.mpc_ip_tolerance_int,
                wp.mpc_ip_tolerance_ext,
                wp.mpc_ip_t,
                wp.mpc_ip_mu,
                wp.mpc_ip_bs_alpha,
                wp.mpc_ip_bs_beta,
                wp.mpc_ip_max_iter,
                (smpc::backtrackingSearchType) wp.mpc_ip_bs_type,
                false); // objective
    }
//    smpc::enable_fexceptions();
}

void oru_walk::initwalkControl()
{
    sleep(1.0);

    init();

    initRobot();

    sleep(3.0);

    wp.walkParametersInitialize(); //wp.readparameters

    readSensors (nao.state_sensor);

    initSolver();
}


/**
 * @brief A control loop, that is executed in separate thread.
 * @attention REAL-TIME!
 */
void oru_walk::walkControl()
{
    //oruw_timer timer(__FUNCTION__, wp.loop_time_limit_ms);
    // support foot position and orientation

    initwalkControl();

    WMG wmg(wp.preview_window_size,
            wp.preview_sampling_time_ms,
            wp.step_height,
            wp.bezier_weight_1,
            wp.bezier_weight_2,
            wp.bezier_inclination_1,
            wp.bezier_inclination_2);
    wmg.T_ms[0] = wp.control_sampling_time_ms;
    wmg.T_ms[1] = wp.control_sampling_time_ms;

    smpc::state_com CoM;

    try
    {
        // steps
        initWalkPattern(wmg);
        // error in position of the swing foot
        correctNextSupportPosition(wmg);
    }
    catch (...)
    {
        return;
    }

    nao.getCoM (nao.state_sensor, nao.CoM_position);

    smpc_parameters mpc(wp.preview_window_size, nao.CoM_position[2]);
    mpc.init_state.set(nao.CoM_position[0], nao.CoM_position[1]);


    jointState target_joint_state = nao.state_model;
    dcm_loop_break = false;
    for (;;)
    {
        //printMessage ("waiting for thread to get unlock");
        boost::unique_lock<boost::mutex> lock(walk_control_mutex);
        walk_control_condition.wait(lock);
        lock.unlock();
        //usleep(100000);
        //readSensors (nao.state_sensor);

        try
        {
            feedbackError (mpc.init_state);
            if (solveMPCProblem (wmg, mpc))  // solve MPC
            {
                //printMessage ("MPC problem solved.");
                if (wmg.isSupportSwitchNeeded())
                {
                    correctNextSupportPosition(wmg);
                    nao.switchSupportFoot();
                }

                // the old solution from is an initial guess;
                solver->get_state(CoM, 0);
                solveIKsendCommands (mpc, CoM, 1, wmg);
                target_joint_state = nao.state_model;
                solver->get_state(CoM, 1);
                solveIKsendCommands (mpc, CoM, 2, wmg);
            }
            else
            {
                //printMessage ("MPC problem not solved. breaking");
                break;
            }

        }
        catch (...)
        {
            printMessage ("walk main control loop. exception caught.");
            break;
        }
    }

    dcm_loop_break = true;

    //ORUW_LOG_STEPS(wmg);
    //ORUW_LOG_CLOSE;
}

void oru_walk::MatrixPrint(unsigned int m, unsigned int n, double * A, const char * description)
{
    unsigned int  i, j;

    printf(" %s", description);
    for (i=0; i<m; i++)
    {
        printf("\n");
        for (j=0; j<n; j++)
            printf("% f ", A[ j*m + i ]);
    }
    printf("\n");
}

/**
 * @brief Solve inverse kinematics and send commands to the controllers.
 *
 * @param[in] mpc MPC parameters
 * @param[in] CoM  CoM position
 * @param[in] control_loop_num number of control loops in future (>= 1).
 * @param[in,out] wmg WMG
 */
void oru_walk::solveIKsendCommands (
        const smpc_parameters &mpc,
        const smpc::state_com &CoM,
        const int control_loop_num,
        WMG &wmg)
{
    // hCoM is constant!
    nao.setCoM(CoM.x(), CoM.y(), mpc.hCoM);

    // support foot and swing foot position/orientation
    wmg.getFeetPositions (
            control_loop_num * wp.control_sampling_time_ms, 
            nao.left_foot_posture.data(), 
            nao.right_foot_posture.data());


    // inverse kinematics
    int iter_num = nao.igm (
            ref_joint_angles, 
            wp.igm_mu, 
            wp.igm_tol, 
            wp.igm_max_iter);
    //ORUW_LOG_MESSAGE("IGM iterations num: %d\n", iter_num);
    if (iter_num < 0)
    {
        halt("IK does not converge.\n", __FUNCTION__);
    }
    int failed_joint = nao.state_model.checkJointBounds();
    if (failed_joint >= 0)
    {
        //ORUW_LOG_MESSAGE("Failed joint: %d\n", failed_joint);
        halt("Joint bounds are violated.\n", __FUNCTION__);
    } else {
        // Set commands
        try
        {   robot_interface_->getTorsoFeedback(torso_feedback_);
            postureController();

            for (int i = 0; i < LOWER_JOINTS_NUM; i++)
            {
                joint_commands[i] = nao.state_model.q[i];
                //ref_joint_angles[i] = nao.state_model.q[i];
            }

            robot_interface_->sendPositionsCommand(joint_indices_, joint_commands, JOINTS_NUM);
        }
        catch (...)
        {
            //ORUW_LOG_MESSAGE("Cannot set joint angles: %s", e.what());
            halt("Cannot set joint angles!", __FUNCTION__);
        }
    }
}

void oru_walk::postureController()
{
    double kp = 0.1;
    double kd = -0.05;
    double delta_pitch = kp*(0.0-torso_feedback_[0]) + kd*(0-torso_feedback_[2]);
    double delta_roll = kp*(0.0-torso_feedback_[1]) + kd*(0-torso_feedback_[3]);;
    nao.state_model.q[R_HIP_PITCH] += delta_pitch;
    nao.state_model.q[L_HIP_PITCH] += delta_pitch;
    nao.state_model.q[R_HIP_ROLL] += delta_roll;
    nao.state_model.q[L_HIP_ROLL] += delta_roll;
}


/**
 * @brief Correct state and the model based on the sensor data.
 *
 * @param[in,out] init_state expected state
 */
void oru_walk::feedbackError (smpc::state_com &init_state)
{
    nao.getCoM (nao.state_sensor, nao.CoM_position);

    smpc::state_com state_error;
    state_error.set (
            init_state.x() - nao.CoM_position[0],
            init_state.y() - nao.CoM_position[1]);

    if (state_error.x() > wp.feedback_threshold)
    {
        state_error.x() -= wp.feedback_threshold;
    }
    else if (state_error.x() < -wp.feedback_threshold)
    {
        state_error.x() += wp.feedback_threshold;
    }
    else
    {
        state_error.x() = 0.0;
    }

    if (state_error.y() > wp.feedback_threshold)
    {
        state_error.y() -= wp.feedback_threshold;
    }
    else if (state_error.y() < -wp.feedback_threshold)
    {
        state_error.y() += wp.feedback_threshold;
    }
    else
    {
        state_error.y() = 0.0;
    }
    //std::cout << "state_error: [" <<  state_error.x() << "," << state_error.y() << "]" << std::endl;
    init_state.x() -= wp.feedback_gain * state_error.x();
    init_state.y() -= wp.feedback_gain * state_error.y();
}



/**
 * @brief Solve the MPC problem.
 *
 * @param[in,out] wmg WMG
 * @param[in,out] mpc MPC parameters
 *
 * @return false if there is not enough steps, true otherwise.
 */
bool oru_walk::solveMPCProblem (
        WMG &wmg,
        smpc_parameters &mpc)
{
    if (wmg.formPreviewWindow(mpc) == WMG_HALT)
    {
        //printMessage("Not enough steps to form preview window. Stopping.");
        return (false);
    }

    //------------------------------------------------------
    solver->set_parameters (mpc.T, mpc.h, mpc.h[0], mpc.angle, mpc.zref_x, mpc.zref_y, mpc.lb, mpc.ub);
    solver->form_init_fp (mpc.fp_x, mpc.fp_y, mpc.init_state, mpc.X);
    solver->solve();
    solver->get_next_state(mpc.init_state);
    //------------------------------------------------------

    return (true);
}


// ==============================================================================
/**
 * @brief Unregister callback and log a message.
 *
 * @param[in] message a message
 */
void oru_walk::printMessage(const char* message)
{
    //ORUW_LOG_MESSAGE("%s", message);
    std::cout << message << std::endl;
    //dcm_callback_connection.disconnect();
}



/**
 * @brief Log a message, remove stiffness and die.
 *
 * @param[in] message a message
 * @param[in] function name of the calling function.
 */
void oru_walk::halt(const char *message, const char* function)
{
    printMessage(message);
    setStiffness(0.0);
    //throw ALERROR(getName(), function, message);
}



/**
 * @brief An interface function that is called remotely to stop
 * the execution.
 */
void oru_walk::stopWalkingRemote()
{
    printMessage ("Stopped by user's request.\n");
    //ORUW_LOG_CLOSE;
}
/*
    /*
    std::vector<std::string> signal_names;
    signal_names.resize(3);
    signal_names[0] = "accel-X";
    signal_names[1] = "accel-Y";
    signal_names[2] = "accel-Z";
    float dummy;

    for (int i=0; i < signal_names.size(); i++) {
        const char *p = signal_names[i].c_str();
        simxGetFloatSignal(clientID, p, &dummy, simx_opmode_streaming);

        std::cout << signal_names[i] << ":" << dummy << std::endl;
    }*/