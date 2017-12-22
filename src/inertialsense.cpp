#include "inertialsense.h"

GPSDriverInertialSense::GPSDriverInertialSense(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
																							 struct vehicle_gps_position_s *gps_position,
																							 struct satellite_info_s *satellite_info)
	: GPSHelper(callback, callback_user)
{}

GPSDriverInertialSense::~GPSDriverInertialSense(){}
int GPSDriverInertialSense::receive(unsigned timeout){}
int GPSDriverInertialSense::configure(unsigned &baud, OutputMode output_mode){}
void GPSDriverInertialSense::setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur){}
int GPSDriverInertialSense::restartSurveyIn(){}
