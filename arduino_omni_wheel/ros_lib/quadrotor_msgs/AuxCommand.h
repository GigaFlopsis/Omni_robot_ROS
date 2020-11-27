#ifndef _ROS_quadrotor_msgs_AuxCommand_h
#define _ROS_quadrotor_msgs_AuxCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace quadrotor_msgs
{

  class AuxCommand : public ros::Msg
  {
    public:
      typedef float _current_yaw_type;
      _current_yaw_type current_yaw;
      typedef float _kf_correction_type;
      _kf_correction_type kf_correction;
      float angle_corrections[2];
      typedef bool _enable_motors_type;
      _enable_motors_type enable_motors;
      typedef bool _use_external_yaw_type;
      _use_external_yaw_type use_external_yaw;

    AuxCommand():
      current_yaw(0),
      kf_correction(0),
      angle_corrections(),
      enable_motors(0),
      use_external_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->current_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->kf_correction);
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_corrections[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_enable_motors;
      u_enable_motors.real = this->enable_motors;
      *(outbuffer + offset + 0) = (u_enable_motors.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable_motors);
      union {
        bool real;
        uint8_t base;
      } u_use_external_yaw;
      u_use_external_yaw.real = this->use_external_yaw;
      *(outbuffer + offset + 0) = (u_use_external_yaw.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_external_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kf_correction));
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_corrections[i]));
      }
      union {
        bool real;
        uint8_t base;
      } u_enable_motors;
      u_enable_motors.base = 0;
      u_enable_motors.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable_motors = u_enable_motors.real;
      offset += sizeof(this->enable_motors);
      union {
        bool real;
        uint8_t base;
      } u_use_external_yaw;
      u_use_external_yaw.base = 0;
      u_use_external_yaw.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_external_yaw = u_use_external_yaw.real;
      offset += sizeof(this->use_external_yaw);
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/AuxCommand"; };
    virtual const char * getMD5() override { return "94f75840e4b1e03675da764692f2c839"; };

  };

}
#endif
