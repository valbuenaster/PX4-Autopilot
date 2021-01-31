/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "output_control.hpp"

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>

using namespace time_literals;


OutputControl::OutputControl(uint8_t max_num_outputs, OutputModuleInterface &interface,
			     SchedulingPolicy scheduling_policy,
			     bool support_esc_calibration, bool ramp_up)
	: ModuleParams(&interface),
	  _control_subs{
	{&interface, ORB_ID(output_control_ca)},
	{&interface, ORB_ID(output_control_mixer)},
	{&interface, ORB_ID(output_control_mavlink)},
	{&interface, ORB_ID(output_control_internal)},
},
_output_module_prefix(interface.get_param_prefix()),
_scheduling_policy(scheduling_policy),
_support_esc_calibration(support_esc_calibration),
_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
_interface(interface),
_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency"))
{
	output_limit_init(&_output_limit);
	_output_limit.ramp_up = ramp_up;

	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	// Enforce the existence of the test_motor topic, so we won't miss initial publications
	test_motor_s test{};
	uORB::Publication<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test);
	_motor_test.test_motor_sub.subscribe();
}

OutputControl::~OutputControl()
{
	perf_free(_control_latency_perf);
	px4_sem_destroy(&_lock);
}

void OutputControl::printStatus() const
{
	perf_print_counter(_control_latency_perf);
	PX4_INFO("Switched to rate_ctrl work queue: %i", (int)_wq_switched);
	// PX4_INFO("Mixer loaded: %s", _mixers ? "yes" : "no");
	PX4_INFO("Driver instance: %i", _driver_instance);

	PX4_INFO("Channel Configuration:");

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		PX4_INFO("Channel %i: value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d, function: %d", i,
			 _current_output_value[i],
			 _failsafe_value[i], _disarmed_value[i], _min_value[i], _max_value[i], _assigned_function[i]);
	}
}

void OutputControl::updateParams()
{
	ModuleParams::updateParams();

	// Load the Mode parameter
	/// TODO: for later: this is the param that will switch between mixer-file
	/// output and parameter-controlled output
	/// Should we have a mode where we allow both to coexist, or should that be the default?
	char pname[16];
	sprintf(pname, "%s_MODE", _output_module_prefix);

	int32_t mode;
	param_get(param_find(pname), &mode);

	if (mode == 0) {
		// Legacy mixer mode; don't run this module?
		return;
	}

	/** Update Function Mappings */

	updateParamValues("FUNC", _assigned_function);
	// updateParamValues("MIN",  _min_value);
	// updateParamValues("MAX",  _max_value);
	updateParamValues("FAIL", _failsafe_value);
	updateParamValues("DIS",  _disarmed_value);
	updateMinValues();
	updateMaxValues();
	// updateParamValues("TRIM", _trim_value);
	updateTrimValues();
	updateReverseMask();

	// Determine the groups required to subscribe to
	_groups_required = 0;

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int32_t func = _assigned_function[i];

		if (func >= output_control_s::FUNCTION_CA0 &&
		    func <= output_control_s::FUNCTION_CA15) {

			_groups_required |= (1 << 0);

		} else if (func == output_control_s::FUNCTION_MIXER) {
			_groups_required |= (1 << 1);

		} else if (func >= output_control_s::FUNCTION_MAVLINK_SERVO0 &&
			   func <= output_control_s::FUNCTION_MAVLINK_SERVO7) {

			_groups_required |= (1 << 2);
			PX4_INFO("Enabling group MAVLINK");

		} else if (func != output_control_s::FUNCTION_NONE) {
			_groups_required |= (1 << 3);

		}
	}
}

bool OutputControl::updateSubscriptions(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (_groups_subscribed == _groups_required) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		// if subscribed to control group 0 or 1 then move to the rate_ctrl WQ
		const bool sub_group_ca = (_groups_required & (1 << 0));
		const bool sub_group_mix = (_groups_required & (1 << 1));

		if (allow_wq_switch && !_wq_switched && (sub_group_ca || sub_group_mix)) {
			if (_interface.ChangeWorkQeue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}

		bool sub_ca_callback_registered = false;
		bool sub_mix_callback_registered = false;

		// register callback to all required actuator control groups
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {

			if (limit_callbacks_to_primary) {
				// don't register additional callbacks if actuator_controls_0 or actuator_controls_1 are already registered
				if ((i > 1) && (sub_ca_callback_registered || sub_mix_callback_registered)) {
					break;
				}
			}

			if (_groups_required & (1 << i)) {
				if (_control_subs[i].registerCallback()) {
					PX4_INFO("subscribed to output_control_%d", i);

					if (limit_callbacks_to_primary) {
						if (i == 0) {
							sub_ca_callback_registered = true;

						} else if (i == 1) {
							sub_mix_callback_registered = true;
						}
					}

				} else {
					PX4_INFO("output_control_%d register callback failed!", i);
				}
			}
		}

		// if nothing required keep periodic schedule (so the module can update other things)
		if (_groups_required == 0) {
			// TODO: this might need to be configurable depending on the module
			_interface.ScheduleOnInterval(100_ms);
		}
	}

	_groups_subscribed = _groups_required;
	setMaxTopicUpdateRate(_max_topic_update_interval_us);

	PX4_INFO("_groups_required 0x%08x", _groups_required);
	PX4_INFO("_groups_subscribed 0x%08x", _groups_subscribed);

	unlock();

	return true;
}

void OutputControl::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	for (unsigned i = 0; i < output_control_s::NUM_OUTPUT_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			_control_subs[i].set_interval_us(_max_topic_update_interval_us);
		}
	}
}

void OutputControl::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_min_value[i] = value;
	}
}

void OutputControl::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_max_value[i] = value;
	}
}

void OutputControl::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_failsafe_value[i] = value;
	}
}

void OutputControl::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_disarmed_value[i] = value;
	}
}

void OutputControl::setTrims(int16_t *values, unsigned nval)
{
	const unsigned N = (nval < MAX_ACTUATORS) ? nval : MAX_ACTUATORS;

	for (unsigned i = 0; i < N; i++) {
		_trim_value[i] = values[i];
	}
}

unsigned OutputControl::getTrims(int16_t *values)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		values[i] = _trim_value[i];
	}

	return MAX_ACTUATORS;
}

void OutputControl::unregister()
{
	for (auto &control_sub : _control_subs) {
		control_sub.unregisterCallback();
	}
}

/**!
 * TODO:
 *
 * 1. Handle slew rate limiting.  Should that apply to the output module specifically, or be handled upstream?
 * 2. Handle "motor test" / general "test" override feature.  Should this be handled at the output module level,
 *    or upstream?
 */

// void OutputControl::updateOutputSlewrateMultirotorMixer()
// {
// 	const hrt_abstime now = hrt_absolute_time();
// 	const float dt = math::constrain((now - _time_last_dt_update_multicopter) / 1e6f, 0.0001f, 0.02f);
// 	_time_last_dt_update_multicopter = now;

// 	// maximum value the outputs of the multirotor mixer are allowed to change in this cycle
// 	// factor 2 is needed because actuator outputs are in the range [-1,1]
// 	const float delta_out_max = 2.0f * 1000.0f * dt / (_max_value[0] - _min_value[0]) / _param_mot_slew_max.get();
// 	_mixers->set_max_delta_out_once(delta_out_max);
// }

// void OutputControl::updateOutputSlewrateSimplemixer()
// {
// 	const hrt_abstime now = hrt_absolute_time();
// 	const float dt = math::constrain((now - _time_last_dt_update_simple_mixer) / 1e6f, 0.0001f, 0.02f);
// 	_time_last_dt_update_simple_mixer = now;

// 	// set dt for slew rate limiter in SimpleMixer (is reset internally after usig it, so needs to be set on every update)
// 	_mixers->set_dt_once(dt);
// }

// unsigned OutputControl::motorTest()
// {
// 	test_motor_s test_motor;
// 	bool had_update = false;

// 	while (_motor_test.test_motor_sub.update(&test_motor)) {
// 		if (test_motor.driver_instance != _driver_instance ||
// 		    test_motor.timestamp == 0 ||
// 		    hrt_elapsed_time(&test_motor.timestamp) > 100_ms) {
// 			continue;
// 		}

// 		bool in_test_mode = test_motor.action == test_motor_s::ACTION_RUN;

// 		if (in_test_mode != _motor_test.in_test_mode) {
// 			// reset all outputs to disarmed on state change
// 			for (int i = 0; i < MAX_ACTUATORS; ++i) {
// 				_current_output_value[i] = _disarmed_value[i];
// 			}
// 		}

// 		if (in_test_mode) {
// 			int idx = test_motor.motor_number;

// 			if (idx < MAX_ACTUATORS) {
// 				if (test_motor.value < 0.f) {
// 					_current_output_value[idx] = _disarmed_value[idx];

// 				} else {
// 					_current_output_value[idx] =
// 						math::constrain<uint16_t>(_min_value[idx] + (uint16_t)((_max_value[idx] - _min_value[idx]) * test_motor.value),
// 									  _min_value[idx], _max_value[idx]);
// 				}
// 			}

// 			if (test_motor.timeout_ms > 0) {
// 				_motor_test.timeout = test_motor.timestamp + test_motor.timeout_ms * 1000;

// 			} else {
// 				_motor_test.timeout = 0;
// 			}
// 		}

// 		_motor_test.in_test_mode = in_test_mode;
// 		had_update = true;
// 	}

// 	// check for timeouts
// 	if (_motor_test.timeout != 0 && hrt_absolute_time() > _motor_test.timeout) {
// 		_motor_test.in_test_mode = false;
// 		_motor_test.timeout = 0;

// 		for (int i = 0; i < MAX_ACTUATORS; ++i) {
// 			_current_output_value[i] = _disarmed_value[i];
// 		}

// 		had_update = true;
// 	}

// 	return (_motor_test.in_test_mode || had_update) ? _max_num_outputs : 0;
// }

bool OutputControl::update()
{
	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;

		if (_armed.armed) {
			_motor_test.in_test_mode = false;
		}
	}

	// // check for motor test
	// if (!_armed.armed && !_armed.manual_lockdown) {
	// 	unsigned num_motor_test = motorTest();

	// 	if (num_motor_test > 0) {
	// 		if (_interface.updateOutputs(false, _current_output_value, num_motor_test, 1)) {
	// 			actuator_outputs_s actuator_outputs{};
	// 			setAndPublishActuatorOutputs(num_motor_test, actuator_outputs);
	// 		}

	// 		// handleCommands();
	// 		return true;
	// 	}
	// }

	if (_param_mot_slew_max.get() > FLT_EPSILON) {
		/// TODO: Implement in ControlAllocation module
		// updateOutputSlewrateMultirotorMixer();
	}

	/// TODO: Implement in ControlAllocation module
	// updateOutputSlewrateSimplemixer(); // update dt for output slew rate in simple mixer

	/// TODO: How to handle ESC calibration with the reorg?

	/* do mixing */
	float outputs[MAX_ACTUATORS] {};

	uint16_t mixed_outputs_mask = 0;

	// Update subscriptions to 'output_control'
	uint8_t grp = 0;
	uint8_t n_updates = 0;

	for (auto &output_sub : _control_subs) {
		if (_groups_required & (1 << grp)) {
			output_control_s controls;

			if (output_sub.update(&controls)) {
				PX4_INFO("Update group %d", grp);
				n_updates++;

				for (uint8_t i = 0; i < controls.n_outputs; i++) {
					auto func = controls.function[i];

					if (func == 0) {
						continue;
					}

					for (uint8_t j = 0; j < _max_num_outputs; j++) {
						if (func == _assigned_function[j]) {
							PX4_INFO("Group %d: Set output %d (func %d) to val %f", grp, j, func, (double)controls.value[i]);
							outputs[j] = controls.value[i];
							mixed_outputs_mask |= (1 << j);
						}
					}
				}
			}
		}

		grp++;
	}

	/* the output limit call takes care of out of band errors, NaN and constrains */
	/// TODO: Can use mixed_outputs_mask to control which channels get updated instead of using the old mixed_outputs_max
	output_limit_calc_mask(_throttle_armed, armNoThrottle(), mixed_outputs_mask, _reverse_output_mask,
			       _disarmed_value, _min_value, _max_value, outputs, _current_output_value, &_output_limit);

	/* overwrite outputs in case of force_failsafe with _failsafe_value values */
	if (_armed.force_failsafe) {
		for (size_t i = 0; i < MAX_ACTUATORS; i++) {
			if (mixed_outputs_mask & (1 << i)) {
				_current_output_value[i] = _failsafe_value[i];
			}
		}
	}

	bool stop_motors = mixed_outputs_mask == 0 || !_throttle_armed;

	/* overwrite outputs in case of lockdown or parachute triggering with disarmed values */
	if (_armed.lockdown || _armed.manual_lockdown) {
		for (size_t i = 0; i < MAX_ACTUATORS; i++) {
			if (mixed_outputs_mask & (1 << i)) {
				_current_output_value[i] = _disarmed_value[i];
			}
		}

		stop_motors = true;
	}

	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, MAX_ACTUATORS,
				     n_updates)) { // mixed_num_outputs -> MAX_ACTUATORS
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(MAX_ACTUATORS, actuator_outputs); // mixed_num_outputs -> MAX_ACTUATORS

		updateLatencyPerfCounter(actuator_outputs);
	}

	// handleCommands();

	return true;
}

void
OutputControl::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);
}

void
OutputControl::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	// use first valid timestamp_sample for latency tracking
	for (int i = 0; i < output_control_s::NUM_OUTPUT_CONTROL_GROUPS; i++) {
		const bool required = _groups_required & (1 << i);
		const hrt_abstime &timestamp_sample =
			_controls[i].timestamp_sample; /// TODO: Ensure this value gets set (timestamp_sample)

		if (required && (timestamp_sample > 0)) {
			perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
			break;
		}
	}
}

void OutputControl::updateParamValues(const char *type, uint16_t values[MAX_ACTUATORS])
{
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_%s%d", _output_module_prefix, type, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);
			values[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, values[i]);
		}
	}
}

void OutputControl::updateFailsafeValues()
{
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_FAIL%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);
			_failsafe_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _failsafe_value[i]);
		}
	}
}

void OutputControl::updateDisarmedValues()
{
	int32_t default_dis;
	param_get(param_find("PWM_DISARMED"), &default_dis); /// TODO: This needs to be per-instance; i.e. different for UAVCAN

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_DIS%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the case of the default -1, use PWM_DISARMED
				PX4_INFO("%d: Defaulting to PWM_DISARMED %d", i, default_dis);
				pval = default_dis;
			}

			_disarmed_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _disarmed_value[i]);
		}
	}
}

void OutputControl::updateMinValues()
{
	int32_t default_min;
	param_get(param_find("PWM_MIN"), &default_min); /// TODO: This needs to be per-instance; i.e. different for UAVCAN

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_MIN%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the case of the default -1, use PWM_MIN
				PX4_INFO("%d: Defaulting to PWM_MIN %d", i, default_min);
				pval = default_min;
			}

			_min_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _min_value[i]);
		}
	}
}

void OutputControl::updateMaxValues()
{
	int32_t default_max;
	param_get(param_find("PWM_MAX"), &default_max); /// TODO: This needs to be per-instance; i.e. different for UAVCAN

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_MAX%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval < 0) {
				// In the case of the default -1, use PWM_MAX
				pval = default_max;
			}

			_max_value[i] = (uint16_t)pval;
			PX4_DEBUG("%s: %d", pname, _max_value[i]);
		}
	}
}

void OutputControl::updateTrimValues()
{
	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the struct from parameters */
		sprintf(pname, "%s_TRIM%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			float pval = 0.0f;
			param_get(param_h, &pval);
			_trim_value[i] = (int16_t)(10000 * pval);
			PX4_DEBUG("%s: %d", pname, _trim_value[i]);
		}
	}
}

void OutputControl::updateReverseMask()
{
	_reverse_output_mask = 0;

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		char pname[16];

		/* fill the channel reverse mask from parameters */
		sprintf(pname, "%s_REV%d", _output_module_prefix, i + 1);
		param_t param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t ival = 0;
			param_get(param_h, &ival);
			_reverse_output_mask |= ((int16_t)(ival != 0)) << i;
		}
	}
}
