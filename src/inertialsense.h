/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
  * @file inertialsense.h
  *
  * InertialSense Serial Protocol Definition
  *
  * @author James Jackson <superjax08@gmail.com
  *
  */

#ifndef INERTIALSENSE_H_
#define INERTIALSENSE_H_

#include "gps_helper.h"
#include "../../definitions.h"

#define GPS_UTC_OFFSET 315964782 // as of 2017

#include "InertialSenseSDK/src/com_manager.h"
#include "InertialSenseSDK/src/data_sets.h"

class GPSDriverInertialSense : public GPSHelper
{
public:
  GPSDriverInertialSense(Interface gpsInterface, GPSCallbackPtr callback, void* callback_user,
                         struct vehicle_gps_position_s *gps_position,
                         satellite_info_s *satellite_info);
  virtual ~GPSDriverInertialSense();
  int receive(unsigned timeout);
  int configure(unsigned &baud, OutputMode output_mode);
  void setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur);
  int restartSurveyIn();

	static int IS_read_wrapper(CMHANDLE cmHandle, int pHandle, unsigned char* readIntoBytes, int numberOfBytes);
	static int IS_write_wrapper(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend);
	static void IS_post_rx_wrapper(CMHANDLE cmHandle, int pHandle, p_data_t* dataRead);
	static void IS_RTCM_rx_wrapper(CMHANDLE cmHandle, com_manager_pass_through_t passThroughType, int pHandle, const unsigned char* data, int dataLength);

private:
	void GPS_callback(gps_t* data);
	void GPS_Info_callback(gps_cno_t* data);

	bool			_got_gps;
	bool		  _got_sat_info;

	struct vehicle_gps_position_s *_gps_position {nullptr};
	struct satellite_info_s *_satellite_info {nullptr};

	gps_abstime start_time;
};


#endif


