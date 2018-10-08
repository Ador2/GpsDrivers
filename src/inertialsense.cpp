#include "inertialsense.h"

using namespace std;

#define IS_WARN(...)		{GPS_WARN(__VA_ARGS__);}
#define IS_DEBUG(...)		{/*GPS_WARN(__VA_ARGS__);*/}

GPSDriverInertialSense::GPSDriverInertialSense(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
                                               struct vehicle_gps_position_s *gps_position,
                                               struct satellite_info_s *satellite_info)
    : GPSBaseStationSupport(callback, callback_user)
	, _gps_position(gps_position)
	, _satellite_info(satellite_info)
{
    (void) gpsInterface;
    start_time = gps_absolute_time();
}

GPSDriverInertialSense::~GPSDriverInertialSense()
{

}

/**
 * configure the device
 * @param baud will be set to the baudrate (output parameter)
 * @return 0 on success, <0 otherwise
 */
int GPSDriverInertialSense::configure(unsigned &baud, OutputMode output_mode)
{
  (void)baud;
  // Set the output_mode
//  uint32_t uINS_sys_config;
  uint32_t uINS_RTK_config;

  if (output_mode == OutputMode::RTCM)
  {
    // output RTCM3 on both ports
    uINS_RTK_config = RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1;
  }


  // Configure the Appropriate sys_config bits for RTCM handling
//  int messageSize = is_comm_set_data(DID_FLASH_CONFIG, OFFSETOF(nvm_flash_cfg_t, sysCfgBits), sizeof(uint32_t), &uINS_sys_config);
  int messageSize = is_comm_set_data(&is_comm_, DID_FLASH_CONFIG, offsetof(nvm_flash_cfg_t, RTKCfgBits), sizeof(uint32_t), &uINS_RTK_config);
  write(message_buffer_, messageSize);

  messageSize = is_comm_get_data_rmc(&is_comm_, RMC_BITS_GPS1_POS | RMC_BITS_GPS1_VEL | RMC_BITS_INS2);
  write(message_buffer_, messageSize);

  if (receive(1000) != 0)
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

void GPSDriverInertialSense::ins2Callback(ins_2_t *data)
{
    float vel_ned[3];
    quatRot(vel_ned, data->qn2b, data->uvw);
    _gps_position->vel_n_m_s = vel_ned[0];
    _gps_position->vel_e_m_s = vel_ned[1];
    _gps_position->vel_d_m_s = vel_ned[2];

    _gps_position->lat = (int32_t)(data->lla[0]*1e7);
    _gps_position->lon = (int32_t)(data->lla[1]*1e7);
    _gps_position->alt = (int32_t)(data->lla[2]*1e3);


    _gps_position->eph = 0.3;
    _gps_position->epv = 0.3;

    float n2 = _gps_position->vel_n_m_s*_gps_position->vel_n_m_s;
    float e2 = _gps_position->vel_e_m_s*_gps_position->vel_e_m_s;
    _gps_position->vel_m_s = pow(n2 + e2 + _gps_position->vel_d_m_s*_gps_position->vel_d_m_s, 0.5);
    _gps_position->vel_ned_valid = true;

    _gps_position->cog_rad = atan2(_gps_position->vel_e_m_s, _gps_position->vel_n_m_s);
    _gps_position->c_variance_rad = (n2*_gps_position->epv + e2*_gps_position->epv)/((n2 + e2)*(n2 + e2));
}

void GPSDriverInertialSense::gpsPosCallback(gps_pos_t *data)
{
  _got_sat_info = true;
  _got_gps = true;

  _gps_position->alt_ellipsoid = (int32_t)(data->hMSL * 1e3f);


  _gps_position->hdop = data->pDop;
  _gps_position->hdop = data->pDop * 1.2f; // This is a hack because the inertialsense doesn't output this directly

  _gps_position->noise_per_ms = 0.0f;
  _gps_position->jamming_indicator = 0.0f;


  // Map fix type to mavlink
  uint32_t fix_type = data->status & GPS_STATUS_FIX_MASK;
  switch(fix_type)
  {
  case (GPS_STATUS_FIX_NONE):
    _gps_position->fix_type = 0;
    break;
  case (GPS_STATUS_FIX_2D):
    _gps_position->fix_type = 2;
    break;
  case (GPS_STATUS_FIX_3D):
    _gps_position->fix_type = 3;
    break;
  default:
    _gps_position->fix_type = 0;
    break;
  }

  _gps_position->timestamp_time_relative = gps_absolute_time() - start_time;
  if (data->week > 0)
  {
    _gps_position->time_utc_usec = GPS_UTC_OFFSET + (uint64_t)(data->week*7*24*3600*1e9f) + (uint64_t)(data->towOffset*1e9);
  }
  _satellite_info->count = _gps_position->satellites_used = data->status & (0x00FF);

  // For now, because we aren't getting a true survey status, let's just use the reported accuracy of the GPS position
  SurveyInStatus status;
  status.duration = _gps_position->timestamp_time_relative/1e6f;
  status.flags = (_gps_position->eph < 2.0f);
  status.mean_accuracy = _gps_position->eph*1000;
  surveyInStatus(status);
}



int GPSDriverInertialSense::receive(unsigned timeout)
{
  uint8_t buf[GPS_READ_BUFFER_SIZE];
  _got_sat_info = false;
  _got_gps = false;

  gps_abstime time_started = gps_absolute_time();

  // Wait for a callback to occur
  while ((gps_absolute_time() < time_started + timeout*1000) && !(_got_gps || _got_sat_info))
  {
    int bytes_read = read(buf, sizeof(buf), timeout);

    for (int i = 0; i < bytes_read; i ++)
    {
        uint32_t message_type = is_comm_parse(&is_comm_, buf[i]);
        switch(message_type)
        {
        case DID_NULL:
            // no message yet
            break;
        case DID_INS_2:
            ins2Callback((ins_2_t*)message_buffer_);
            return 0b01;
        case DID_GPS1_POS:
            gpsPosCallback((gps_pos_t*)message_buffer_);
            return 0b11;
            return 0;
            break;
        default:
            break;
        }
    }

  }
  uint8_t ret = 0;

  if (_got_gps)
  {
    ret |= 0b1;
  }

  if(_got_sat_info)
  {
    ret |= 0b10;
  }

  return ret;
}

void GPSDriverInertialSense::setSurveyInSpecs(uint32_t survey_in_acc_limit, uint32_t survey_in_min_dur)
{
  // TODO: We still  have to enable this on the InertialSense
    (void) survey_in_acc_limit;
    (void) survey_in_min_dur;
}

/**
        * Start or restart the survey-in procees. This is only used in RTCM ouput mode.
        * It will be called automatically after configuring.
        * @return 0 on success, <0 on error
        */
int GPSDriverInertialSense::restartSurveyIn(){ return 0; }
