/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file CatapultLaunchMethod.cpp
 * Catapult Launch detection
 * 弹射模式飞行启动检测
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 */

#include "CatapultLaunchMethod.h"

#include <px4_log.h>

namespace launchdetection
{

CatapultLaunchMethod::CatapultLaunchMethod(ModuleParams *parent) :
	ModuleParams(parent)
{
	_last_timestamp = hrt_absolute_time();
}

void CatapultLaunchMethod::update(float accel_x)
{
	float dt = hrt_elapsed_time(&_last_timestamp) * 1e-6f;
	_last_timestamp = hrt_absolute_time();

	switch (state) {
	case LAUNCHDETECTION_RES_NONE:

		/* Detect a acceleration that is longer and stronger as the minimum given by the params */
        //检测当前加速度值与起飞加速度阈值的关系，判断是否进入起飞模式
        if (accel_x > _thresholdAccel.get()) {
			_integrator += dt;

            //判断是否满足时间阈值
            if (_integrator > _thresholdTime.get()) {
				if (_motorDelay.get() > 0.0f) {
					state = LAUNCHDETECTION_RES_DETECTED_ENABLECONTROL;
                    //带延迟全速启动电机
					PX4_WARN("Launch detected: enablecontrol, waiting %8.4fs until full throttle",
						 double(_motorDelay.get()));

				} else {
					/* No motor delay set: go directly to enablemotors state */
					state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
                    //不带延迟直接启动电机
					PX4_WARN("Launch detected: enablemotors (delay not activated)");
				}
			}

		} else {
			reset();
		}

		break;

	case LAUNCHDETECTION_RES_DETECTED_ENABLECONTROL:
		/* Vehicle is currently controlling attitude but not with full throttle. Waiting until delay is
		 * over to allow full throttle */
        //飞机当前可以控制姿态，但油门不是全速，延迟过后变为全速
		_motorDelayCounter += dt;

		if (_motorDelayCounter > _motorDelay.get()) {
			PX4_INFO("Launch detected: state enablemotors");
			state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
		}

		break;

	default:
		break;

	}
}

LaunchDetectionResult CatapultLaunchMethod::getLaunchDetected() const
{
	return state;
}

void CatapultLaunchMethod::reset()
{
    //初始化，重新等待启动检测
	_integrator = 0.0f;
	_motorDelayCounter = 0.0f;
	state = LAUNCHDETECTION_RES_NONE;
}

float CatapultLaunchMethod::getPitchMax(float pitchMaxDefault)
{
	/* If motor is turned on do not impose the extra limit on maximum pitch */
    //选择是否在启动时限制俯仰轴最大油门量
	if (state == LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
		return pitchMaxDefault;

	} else {
		return _pitchMaxPreThrottle.get();
	}
}

} // namespace launchdetection
