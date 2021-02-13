/****************************************************************************
 *
 *   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file integrater_accel_main.cpp
 * Implementation of basic integration for Volansi's interview.
 *
 * @author Luis Ariel Valbuena Reyes
 */
/*
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
*/

//Same libraries that ekf2
#include <float.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/integrated_accel.h>
#include <uORB/topics/actuator_armed.h>

#define MICRO_S_2_SECOND 0.000001

extern "C" __EXPORT int integrated_accel_main(int argc, char *argv[]);

class integrated_accel final : public ModuleBase<integrated_accel>, public ModuleParams, public px4::WorkItem
{
public:
    integrated_accel(bool replay_mode);
    ~integrated_accel();

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    /** @see ModuleBase */

    void Run() override;
    bool init();
    int print_status() override;

private:
    bool _replay_mode;
    hrt_abstime now;
    hrt_abstime accelerometer_integral_dt;
    double accel_x;
    double accel_y;
    double accel_z;
    double _integrated_accel_x;
    double _integrated_accel_y;
    double _integrated_accel_z;

    uORB::SubscriptionCallbackWorkItem _sensors_sub{this, ORB_ID(sensor_combined)};
    uORB::SubscriptionCallbackWorkItem _actuator_armed_sub{this, ORB_ID(actuator_armed)};

    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    //uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

    uORB::Publication<integrated_accel_s> _integrated_accel_pub{ORB_ID(integrated_accel)};

};

integrated_accel::integrated_accel(bool replay_mode):ModuleParams(nullptr),
                                                     WorkItem(MODULE_NAME, px4::wq_configurations::integrated_accel),
                                                     _replay_mode(replay_mode),
                                                     now(0),
                                                     accelerometer_integral_dt(0),
                                                     accel_x(0.0f),
                                                     accel_y(0.0f),
                                                     accel_z(0.0f),
                                                     _integrated_accel_x(0.0f),
                                                     _integrated_accel_y(0.0f),
                                                     _integrated_accel_z(0.0f)
{
    updateParams();
}

integrated_accel::~integrated_accel()
{

}

bool integrated_accel::init()
{
    if (!_sensors_sub.registerCallback()) {
        PX4_ERR("sensor combined callback registration failed!");
        return false;
    }
    if (!_actuator_armed_sub.registerCallback()) {
        PX4_ERR("actuator armed callback registration failed!");
        return false;
    }
    PX4_INFO("the program entered integrated_accel::init()");

    return true;
}

void integrated_accel::Run()
{
    if (should_exit())
    {
        _sensors_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }
    sensor_combined_s sensors;
    actuator_armed_s ActuatorA;

    if(_actuator_armed_sub.update(&ActuatorA))
    {
        if(_sensors_sub.update(&sensors))
        {
            if(_parameter_update_sub.updated())
            {
                parameter_update_s pupdate;
                _parameter_update_sub.copy(&pupdate);

                updateParams();
            }

            struct integrated_accel_s integratedValues;
            integratedValues.armed = ActuatorA.armed;

            if(integratedValues.armed)
            {
                now = sensors.timestamp;
                accelerometer_integral_dt = sensors.accelerometer_timestamp_relative;//In microseconds
                accel_x = sensors.accelerometer_m_s2[0];
                accel_y = sensors.accelerometer_m_s2[1];
                accel_z = sensors.accelerometer_m_s2[2];

                if(accelerometer_integral_dt == 0)
                {
                    //PX4_INFO("accelerometer_integral_dt == 0, set to 1");
                    accelerometer_integral_dt = 100000;//100 ms
                }

                _integrated_accel_x += accel_x * accelerometer_integral_dt * MICRO_S_2_SECOND;
                _integrated_accel_y += accel_y * accelerometer_integral_dt * MICRO_S_2_SECOND;
                _integrated_accel_z += accel_z * accelerometer_integral_dt * MICRO_S_2_SECOND;

                //PX4_INFO("Readings: \t%lu,\t %8.4f, \t %8.4f, \t %8.4f.",now,accel_x,accel_y,accel_z);
                integratedValues.integral_dt = accelerometer_integral_dt;
                integratedValues.integrated_x = _integrated_accel_x;
                integratedValues.integrated_y = _integrated_accel_y;
                integratedValues.integrated_z = _integrated_accel_z;
                integratedValues.timestamp = now;

                 _integrated_accel_pub.publish(integratedValues);
            }
        }
    }
}

int integrated_accel::print_status()
{
    PX4_INFO("This should print a status");
    //perf_print_counter(_ekf_update_perf);

    return 0;
}

int integrated_accel_main(int argc, char *argv[])
{
    return integrated_accel::main(argc,argv);
}

int integrated_accel::task_spawn(int argc, char *argv[])
{
    bool replay_mode = false;
    if (argc > 1 && !strcmp(argv[1], "-r")) {
        PX4_INFO("replay mode enabled");
        replay_mode = true;
    }

    integrated_accel *instance = new integrated_accel(replay_mode);

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int integrated_accel::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int integrated_accel::print_usage(const char *reason)
{
    if (reason) {
            PX4_WARN("%s\n", reason);
        }

        PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
    ### Description
    Rough integrator for take-home interview at Volansi.

    The integrator is of the form integrated_value += accel_m_s2 * integral_dt.​

    integrated_accel should be capable to start in replay mode (`-r`): in this mode it does not access the system time, but only uses the
    timestamps from the sensor topics.

    )DESCR_STR");

        PRINT_MODULE_USAGE_NAME("integrated_accel", "rough integrator");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('r', "Enable replay mode", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

        return 0;
}
