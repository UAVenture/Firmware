/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mixer_simple.cpp
 *
 * Simple summing mixer.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

#define SCALER_TABLE_SIZE	4
static mixer_scaler_s		_scaler_table[SCALER_TABLE_SIZE];
static uint8_t			_count;

SimpleMixer::SimpleMixer(ControlCallback control_cb,
			 uintptr_t cb_handle,
			 mixer_simple_s *mixinfo) :
	Mixer(control_cb, cb_handle),
	_info(mixinfo)
{
	_count = 0;

	for (int i = 0; i < SCALER_TABLE_SIZE; i++) {
		_scaler_table[i] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	}
}

SimpleMixer::~SimpleMixer()
{
	if (_info != nullptr) {
		free(_info);
	}
}

int
SimpleMixer::parse_output_scaler(const char *buf, unsigned &buflen, uint8_t &scaler_index)
{
	int ret;
	int s[5];
	int n = -1;

	buf = findtag(buf, buflen, 'O');

	if ((buf == nullptr) || (buflen < 12)) {
		debug("output parser failed finding tag, ret: '%s'", buf);
		return -1;
	}

	if ((ret = sscanf(buf, "O: %d %d %d %d %d %n",
			  &s[0], &s[1], &s[2], &s[3], &s[4], &n)) != 5) {
		debug("out scaler parse failed on '%s' (got %d, consumed %d)", buf, ret, n);
		return -1;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		return -1;
	}

	scaler_index = _count;

	struct mixer_scaler_s scaler;
	scaler.negative_scale	= s[0] / 10000.0f;
	scaler.positive_scale	= s[1] / 10000.0f;
	scaler.offset		= s[2] / 10000.0f;
	scaler.min_output	= s[3] / 10000.0f;
	scaler.max_output	= s[4] / 10000.0f;

	if (!save_scaler_entry(scaler, scaler_index)) {
		debug("scaler table overflow.");
		return -1;
	}

	return 0;
}

int
SimpleMixer::parse_control_scaler(const char *buf, unsigned &buflen, uint8_t &scaler_index, uint8_t &control_group,
				  uint8_t &control_index)
{
	unsigned u[2];
	int s[5];

	buf = findtag(buf, buflen, 'S');

	if ((buf == nullptr) || (buflen < 16)) {
		debug("control parser failed finding tag, ret: '%s'", buf);
		return -1;
	}

	if (sscanf(buf, "S: %u %u %d %d %d %d %d",
		   &u[0], &u[1], &s[0], &s[1], &s[2], &s[3], &s[4]) != 7) {
		debug("control parse failed on '%s'", buf);
		return -1;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		return -1;
	}

	control_group		= u[0];
	control_index		= u[1];

	struct mixer_scaler_s scaler;
	scaler.negative_scale	= s[0] / 10000.0f;
	scaler.positive_scale	= s[1] / 10000.0f;
	scaler.offset		= s[2] / 10000.0f;
	scaler.min_output	= s[3] / 10000.0f;
	scaler.max_output	= s[4] / 10000.0f;

	if (!save_scaler_entry(scaler, scaler_index)) {
		debug("scaler table overflow.");
		return -1;
	}

	return 0;
}

bool
SimpleMixer::save_scaler_entry(mixer_scaler_s &scaler, uint8_t &index) {
	for (int i = 0; i < _count; i++) {
		if (is_equal(_scaler_table[i].negative_scale, scaler.negative_scale) && 
				is_equal(_scaler_table[i].positive_scale, scaler.positive_scale) &&
				is_equal(_scaler_table[i].offset, scaler.offset) &&
				is_equal(_scaler_table[i].min_output, scaler.min_output) &&
				is_equal(_scaler_table[i].max_output, scaler.max_output)) {
			index = i;
			return true;
		}
	}
	
	// WARNING WARNING: CHECK WE DON'T EXCEED THE LIMITS OF THE ARRAY!
	if (_count >= SCALER_TABLE_SIZE) {
		return false;
	}

	index = _count;
	_scaler_table[_count++] = scaler;

	return true;
}

bool
SimpleMixer::is_equal(float var, float value) {
	return (var > value - 0.0001f && var < value + 0.0001f);
}

SimpleMixer *
SimpleMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	SimpleMixer *sm = nullptr;
	mixer_simple_s *mixinfo = nullptr;
	unsigned inputs;
	int used;
	const char *end = buf + buflen;

	/* get the base info for the mixer */
	if (sscanf(buf, "M: %u%n", &inputs, &used) != 1) {
		debug("simple parse failed on '%s'", buf);
		goto out;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		goto out;
	}

	mixinfo = (mixer_simple_s *)malloc(MIXER_SIMPLE_SIZE(inputs));

	if (mixinfo == nullptr) {
		debug("could not allocate memory for mixer info");
		goto out;
	}

	mixinfo->control_count = inputs;

	if (parse_output_scaler(end - buflen, buflen, mixinfo->output_scaler_index)) {
		debug("simple mixer parser failed parsing out scaler tag, ret: '%s'", buf);
		goto out;
	}

	for (unsigned i = 0; i < inputs; i++) {
		if (parse_control_scaler(end - buflen, buflen,
					 mixinfo->controls[i].scaler_index,
					 mixinfo->controls[i].control_group,
					 mixinfo->controls[i].control_index)) {
			debug("simple mixer parser failed parsing ctrl scaler tag, ret: '%s'", buf);
			goto out;
		}

	}

	warnx("COUNT %d", _count);

	sm = new SimpleMixer(control_cb, cb_handle, mixinfo);

	if (sm != nullptr) {
		mixinfo = nullptr;
		debug("loaded mixer with %d input(s)", inputs);

	} else {
		debug("could not allocate memory for mixer");
	}

out:

	if (mixinfo != nullptr) {
		free(mixinfo);
	}

	return sm;
}

SimpleMixer *
SimpleMixer::pwm_input(Mixer::ControlCallback control_cb, uintptr_t cb_handle, unsigned input, uint16_t min,
		       uint16_t mid, uint16_t max)
{
	SimpleMixer *sm = nullptr;
	mixer_simple_s *mixinfo = nullptr;

	mixinfo = (mixer_simple_s *)malloc(MIXER_SIMPLE_SIZE(1));

	if (mixinfo == nullptr) {
		debug("could not allocate memory for mixer info");
		goto out;
	}

	mixinfo->control_count = 1;

	/*
	 * Always pull from group 0, with the input value giving the channel.
	 */
	mixinfo->controls[0].control_group = 0;
	mixinfo->controls[0].control_index = input;

	/*
	 * Conversion uses both the input and output side of the mixer.
	 *
	 * The input side is used to slide the control value such that the min argument
	 * results in a value of zero.
	 *
	 * The output side is used to apply the scaling for the min/max values so that
	 * the resulting output is a -1.0 ... 1.0 value for the min...max range.
	 
	mixinfo->controls[0].scaler.negative_scale = 1.0f;
	mixinfo->controls[0].scaler.positive_scale = 1.0f;
	mixinfo->controls[0].scaler.offset = -mid;
	mixinfo->controls[0].scaler.min_output = -(mid - min);
	mixinfo->controls[0].scaler.max_output = (max - mid);

	mixinfo->output_scaler.negative_scale = 500.0f / (mid - min);
	mixinfo->output_scaler.positive_scale = 500.0f / (max - mid);
	mixinfo->output_scaler.offset = 0.0f;
	mixinfo->output_scaler.min_output = -1.0f;
	mixinfo->output_scaler.max_output = 1.0f;
*/
	sm = new SimpleMixer(control_cb, cb_handle, mixinfo);

	if (sm != nullptr) {
		mixinfo = nullptr;
		debug("PWM input mixer for %d", input);

	} else {
		debug("could not allocate memory for PWM input mixer");
	}

out:

	if (mixinfo != nullptr) {
		free(mixinfo);
	}

	return sm;
}

unsigned
SimpleMixer::mix(float *outputs, unsigned space, uint16_t *status_reg)
{
	float		sum = 0.0f;

	if (_info == nullptr) {
		return 0;
	}

	if (space < 1) {
		return 0;
	}

	for (unsigned i = 0; i < _info->control_count; i++) {
		float input;

		_control_cb(_cb_handle,
			    _info->controls[i].control_group,
			    _info->controls[i].control_index,
			    input);

		sum += scale(_scaler_table[_info->controls[i].scaler_index], input);
	}

	*outputs = scale(_scaler_table[_info->output_scaler_index], sum);
	return 1;
}

void
SimpleMixer::groups_required(uint32_t &groups)
{
	for (unsigned i = 0; i < _info->control_count; i++) {
		groups |= 1 << _info->controls[i].control_group;
	}
}

int
SimpleMixer::check()
{
	int ret;
	float junk;

	/* sanity that presumes that a mixer includes a control no more than once */
	/* max of 32 groups due to groups_required API */
	if (_info->control_count > 32) {
		return -2;
	}

	/* validate the output scaler */
	ret = scale_check(_scaler_table[_info->output_scaler_index]);

	ret = (_scaler_table[_info->output_scaler_index].negative_scale >= 1.0f && _scaler_table[_info->output_scaler_index].negative_scale <= 1.01f ? 0 : 1);

	if (ret != 0) {
		return ret;
	}

	/* validate input scalers */
	for (unsigned i = 0; i < _info->control_count; i++) {

		/* verify that we can fetch the control */
		if (_control_cb(_cb_handle,
				_info->controls[i].control_group,
				_info->controls[i].control_index,
				junk) != 0) {
			return -3;
		}

		/* validate the scaler */
		ret = scale_check(_scaler_table[_info->controls[i].scaler_index]);

		if (ret != 0) {
			return (10 * i + ret);
		}
	}

	return 0;
}
