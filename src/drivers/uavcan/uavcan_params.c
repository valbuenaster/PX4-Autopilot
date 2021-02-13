/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

/**
 * UAVCAN mode
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables support for UAVCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for UAVCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for UAVCAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to UAVCAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

/**
 * UAVCAN Node ID.
 *
 * Read the specs at http://uavcan.org to learn more about Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_NODE_ID, 1);

/**
 * UAVCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_BITRATE, 1000000);

/**
 * UAVCAN ESC will spin at idle throttle when armed, even if the mixer outputs zero setpoints.
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ESC_IDLT, 1);

/**
 * UAVCAN rangefinder minimum range
 *
 * This parameter defines the minimum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MIN, 0.3f);

/**
 * UAVCAN rangefinder maximum range
 *
 * This parameter defines the maximum valid range for a rangefinder connected via UAVCAN.
 *
 * @unit m
 * @group UAVCAN
 */
PARAM_DEFINE_FLOAT(UAVCAN_RNG_MAX, 200.0f);

/**
 * UAVCAN ANTI_COLLISION light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the ANTI_COLLISION lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_ANTCL, 2);

/**
 * UAVCAN STROBE light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the STROBE lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_STROB, 1);

/**
 * UAVCAN RIGHT_OF_WAY light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the RIGHT_OF_WAY lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_NAV, 3);

/**
 * UAVCAN LIGHT_ID_LANDING light operating mode
 *
 * This parameter defines the minimum condition under which the system will command
 * the LIGHT_ID_LANDING lights on
 *
 *  0 - Always off
 *  1 - When autopilot is armed
 *  2 - When autopilot is prearmed
 *  3 - Always on
 *
 * @min 0
 * @max 3
 * @value 0 Always off
 * @value 1 When autopilot is armed
 * @value 2 When autopilot is prearmed
 * @value 3 Always on
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_LGT_LAND, 0);

/******************************************************************************
*                                 CAN_ACT_FUNC                                *
******************************************************************************/
/**
 * Actuator function assigned to CAN1
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC1, 0);

/**
 * Actuator function assigned to CAN2
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC2, 0);

/**
 * Actuator function assigned to CAN3
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC3, 0);

/**
 * Actuator function assigned to CAN4
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC4, 0);

/**
 * Actuator function assigned to CAN5
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC5, 0);

/**
 * Actuator function assigned to CAN6
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC6, 0);

/**
 * Actuator function assigned to CAN7
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC7, 0);

/**
 * Actuator function assigned to CAN8
 * See output_control FUNCTION values
 *
 * @reboot_required false
 * @min 0
 * @max 32768
 * @group PWM
 * @value 0 NONE
 */
PARAM_DEFINE_INT32(CAN_ACT_FUNC8, 0);

/******************************************************************************
*                                 CAN_ACT_MIN                                *
******************************************************************************/
/**
 * Set the default MIN output value
 *
 * @reboot_required false
 *
 * @min -8191
 * @max 8191
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN, 1);

/**
 * Set the min PWM value for the main 1 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required false
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN1, -1);

/**
 * Set the min PWM value for the main 2 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN2, -1);

/**
 * Set the min PWM value for the main 3 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN3, -1);

/**
 * Set the min PWM value for the main 4 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN4, -1);

/**
 * Set the min PWM value for the main 5 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN5, -1);

/**
 * Set the min PWM value for the main 6 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN6, -1);

/**
 * Set the min PWM value for the main 7 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN7, -1);

/**
 * Set the min PWM value for the main 8 output
 *
 * This is the minimum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MIN will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MIN8, -1);

/******************************************************************************
*                                 CAN_ACT_MAX                                *
******************************************************************************/
/**
 * Set the default MAX output value
 *
 * @reboot_required false
 *
 * @min -8191
 * @max 8191
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX, 8191);

/**
 * Set the max PWM value for the main 1 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX1, -1);

/**
 * Set the max PWM value for the main 2 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX2, -1);

/**
 * Set the max PWM value for the main 3 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX3, -1);

/**
 * Set the max PWM value for the main 4 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX4, -1);

/**
 * Set the max PWM value for the main 5 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX5, -1);

/**
 * Set the max PWM value for the main 6 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX6, -1);

/**
 * Set the max PWM value for the main 7 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX7, -1);

/**
 * Set the max PWM value for the main 8 output
 *
 * This is the maximum PWM pulse the autopilot is allowed to output.
 * When set to -1 the value for PWM_MAX will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_MAX8, -1);

/******************************************************************************
*                                CAN_ACT_FAIL                                *
******************************************************************************/
/**
 * Set the default FAILSAFE output value
 *
 * @reboot_required false
 *
 * @min -8191
 * @max 8191
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL, 0);

/**
 * Set the failsafe PWM for the main 1 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL1, -1);

/**
 * Set the failsafe PWM for the main 2 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL2, -1);

/**
 * Set the failsafe PWM for the main 3 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL3, -1);

/**
 * Set the failsafe PWM for the main 4 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL4, -1);

/**
 * Set the failsafe PWM for the main 5 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL5, -1);

/**
 * Set the failsafe PWM for the main 6 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL6, -1);

/**
 * Set the failsafe PWM for the main 7 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL7, -1);

/**
 * Set the failsafe PWM for the main 8 output
 *
 * This is the PWM pulse the autopilot is outputting if in failsafe mode.
 * When set to -1 the value is set automatically depending if the actuator
 * is a motor (900us) or a servo (1500us)
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_FAIL8, -1);

/******************************************************************************
*                                CAN_ACT_DIS                                 *
******************************************************************************/
/**
 * Set the default DISARMED output value
 *
 * @reboot_required false
 *
 * @min -8191
 * @max 8191
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS, 0);

/**
 * Set the disarmed PWM for the main 1 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS1, -1);

/**
 * Set the disarmed PWM for the main 2 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS2, -1);

/**
 * Set the disarmed PWM for the main 3 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS3, -1);

/**
 * Set the disarmed PWM for the main 4 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS4, -1);

/**
 * Set the disarmed PWM for the main 5 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS5, -1);

/**
 * Set the disarmed PWM for the main 6 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS6, -1);

/**
 * Set the disarmed PWM for the main 7 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS7, -1);

/**
 * Set the disarmed PWM for the main 8 output
 *
 * This is the PWM pulse the autopilot is outputting if not armed.
 * When set to -1 the value for PWM_DISARMED will be used
 *
 * @reboot_required true
 *
 * @min -8191
 * @max 8191
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_DIS8, -1);

/******************************************************************************
*                                CAN_ACT_REV                                 *
******************************************************************************/

/**
 * Invert direction of main output channel 1
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV1, 0);

/**
 * Invert direction of main output channel 2
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV2, 0);

/**
 * Invert direction of main output channel 3
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV3, 0);

/**
 * Invert direction of main output channel 4
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV4, 0);

/**
 * Invert direction of main output channel 5
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV5, 0);

/**
 * Invert direction of main output channel 6
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV6, 0);

/**
 * Invert direction of main output channel 7
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV7, 0);

/**
 * Invert direction of main output channel 8
 *
 * Enable to invert the channel.
 * Warning: Use this parameter when connected to a servo only.
 * For a brushless motor, invert manually two phases to reverse the direction.
 *
 * @boolean
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(CAN_ACT_REV8, 0);

/******************************************************************************
*                                CAN_ACT_TRIM                                *
******************************************************************************/

/**
 * Trim value for main output channel 1
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM1, 0);

/**
 * Trim value for main output channel 2
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM2, 0);

/**
 * Trim value for main output channel 3
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM3, 0);

/**
 * Trim value for main output channel 4
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM4, 0);

/**
 * Trim value for main output channel 5
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM5, 0);

/**
 * Trim value for main output channel 6
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM6, 0);

/**
 * Trim value for main output channel 7
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM7, 0);

/**
 * Trim value for main output channel 8
 *
 * Set to normalized offset
 *
 * @min -0.2
 * @max 0.2
 * @decimal 2
 * @group PWM Outputs
 */
PARAM_DEFINE_FLOAT(CAN_ACT_TRIM8, 0);
