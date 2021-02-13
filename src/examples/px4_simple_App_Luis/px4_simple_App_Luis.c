/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_App.c
 * Minimal application example for PX4 autopilot
 * followed by 
 * @author Luis Valbuena <valbuenaster@gmail.com>
 */

//#include <px4_platform_common/log.h> //Does not find it
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_armed.h>

#define UPDATE_ACCEL_READING 100
#define UPDATE_ARMED_CHECKING 100


__EXPORT int px4_simple_App_Luis_main(int argc, char *argv[]);

int px4_simple_App_Luis_main(int argc, char *argv[])
{
    bool enable_parameter = false;
    int counter = 0;

    PX4_INFO("Luis Valbuena testing how to code here");

    //Subscribing to a topic
    int sensor_sub_Handler = orb_subscribe(ORB_ID(sensor_combined));
    int accel_sub_Handler = orb_subscribe(ORB_ID(sensor_accel));
    int aircraft_armed_sub_Handler = orb_subscribe(ORB_ID(actuator_armed));

    //To advertise an altitude topic
    struct vehicle_attitude_s att;
    memset(&att,0,sizeof(att));
    orb_advert_t att_pub_Handler = orb_advertise(ORB_ID(vehicle_attitude), &att);

    px4_pollfd_struct_t fds[] = { { .fd = sensor_sub_Handler, .events = POLLIN},
                                  { .fd = accel_sub_Handler, .events = POLLIN},
                                  { .fd = aircraft_armed_sub_Handler, .events = POLLIN},};

    if(argc == 2)
    {
        char *buffer = argv[1];
        //PX4_INFO("buffer[1] = %c", buffer[0]);

        if( buffer[0] > '0' ) enable_parameter = true;

        if(enable_parameter)
        {
            //This limits the handler update at 5 Hz ==> 200 ms
            orb_set_interval(sensor_sub_Handler,200);//200 ms

            //This limits the handler update at 10 Hz ==> 100 ms
            orb_set_interval(accel_sub_Handler,UPDATE_ACCEL_READING);
            orb_set_interval(aircraft_armed_sub_Handler,UPDATE_ARMED_CHECKING);


            while(counter < 500)//THIS SHOULD HAVE ANOTHER CONDITION
            {
                int poll_ret = px4_poll(fds,3,1000);

                if(poll_ret == 0)
                {
                    PX4_INFO("I am not getting data within a second");
                }

                if(fds[0].revents & POLLIN)
                {
                    struct sensor_combined_s rawData;
                    orb_copy(ORB_ID(sensor_combined),sensor_sub_Handler,&rawData);
                    /*
                    PX4_INFO("Accelerometer: \t%8.4f, \t%8.4f, \t%8.4f",
                             (double) rawData.accelerometer_m_s2[0],
                             (double) rawData.accelerometer_m_s2[1],
                             (double) rawData.accelerometer_m_s2[2]);
                    */
                    att.q[0] = rawData.accelerometer_m_s2[0];
                    att.q[1] = rawData.accelerometer_m_s2[1];
                    att.q[2] = rawData.accelerometer_m_s2[2];

                    orb_publish(ORB_ID(vehicle_attitude),att_pub_Handler,&att);
                }

                if(fds[1].revents & POLLIN)
                {
                    struct sensor_accel_s measurements;
                    orb_copy(ORB_ID(sensor_accel),accel_sub_Handler,&measurements);

                    unsigned int timestamp = measurements.timestamp;	// time since system start (microseconds)
                    unsigned int device_id = measurements.device_id; // unique device ID for the sensor that does not change between power cycles
                    unsigned int error_count = measurements.error_count;

                    double x = measurements.x;  // acceleration in the NED X board axis in m/s^2
                    double y = measurements.y;  // acceleration in the NED Y board axis in m/s^2
                    double z = measurements.z;  // acceleration in the NED Z board axis in m/s^2

                    unsigned int integral_dt = measurements.integral_dt;  	//integration time  (microseconds)
                    double x_integral = measurements.x_integral;  // delta velocity in the NED X board axis in m/s over the integration time frame (integral_dt)
                    double y_integral = measurements.y_integral;  // delta velocity in the NED Y board axis in m/s over the integration time frame (integral_dt)
                    double z_integral = measurements.z_integral;  // delta velocity in the NED Z board axis in m/s over the integration time frame (integral_dt)

                    double temperature = measurements.temperature;	// temperature in degrees celsius

                    double scaling = measurements.scaling;  // scaling from raw to m/s^s
                    int x_raw = measurements.x_raw;
                    int y_raw = measurements.y_raw;
                    int z_raw = measurements.z_raw;

                    PX4_INFO("Readings from the sensor_accel topic : \t%i, \t%i, \t%i, \t%8.4f, \t%8.4f, \t%8.4f, \t%i, \t%8.4f, \t%8.4f, \t%8.4f, \t%8.4f, \t%8.4f, \t%i, \t%i, \t%i",
                             timestamp,device_id,error_count,x,y,z,integral_dt,x_integral,y_integral,z_integral,temperature,scaling,x_raw,y_raw,z_raw);

                }


                if(fds[2].revents & POLLIN)
                {
                    bool isArmed = false;
                    struct actuator_armed_s armedCheck;
                    orb_copy(ORB_ID(actuator_armed),aircraft_armed_sub_Handler,&armedCheck);

                    isArmed = armedCheck.armed;

                    if(isArmed)
                    {
                        PX4_INFO("It is Armed");
                    }else{
                        PX4_INFO("It is NOT Armed");
                    }
                }

                counter++;
            }
        }
    }

    PX4_INFO("Getting out");
	return OK;
}

