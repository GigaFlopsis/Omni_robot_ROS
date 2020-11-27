#ifndef _ROS_quadrotor_msgs_PPROutputData_h
#define _ROS_quadrotor_msgs_PPROutputData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrotor_msgs
{

  class PPROutputData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _quad_time_type;
      _quad_time_type quad_time;
      typedef float _des_thrust_type;
      _des_thrust_type des_thrust;
      typedef float _des_roll_type;
      _des_roll_type des_roll;
      typedef float _des_pitch_type;
      _des_pitch_type des_pitch;
      typedef float _des_yaw_type;
      _des_yaw_type des_yaw;
      typedef float _est_roll_type;
      _est_roll_type est_roll;
      typedef float _est_pitch_type;
      _est_pitch_type est_pitch;
      typedef float _est_yaw_type;
      _est_yaw_type est_yaw;
      typedef float _est_angvel_x_type;
      _est_angvel_x_type est_angvel_x;
      typedef float _est_angvel_y_type;
      _est_angvel_y_type est_angvel_y;
      typedef float _est_angvel_z_type;
      _est_angvel_z_type est_angvel_z;
      typedef float _est_acc_x_type;
      _est_acc_x_type est_acc_x;
      typedef float _est_acc_y_type;
      _est_acc_y_type est_acc_y;
      typedef float _est_acc_z_type;
      _est_acc_z_type est_acc_z;
      uint16_t pwm[4];

    PPROutputData():
      header(),
      quad_time(0),
      des_thrust(0),
      des_roll(0),
      des_pitch(0),
      des_yaw(0),
      est_roll(0),
      est_pitch(0),
      est_yaw(0),
      est_angvel_x(0),
      est_angvel_y(0),
      est_angvel_z(0),
      est_acc_x(0),
      est_acc_y(0),
      est_acc_z(0),
      pwm()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->quad_time >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->quad_time >> (8 * 1)) & 0xFF;
      offset += sizeof(this->quad_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->des_thrust);
      offset += serializeAvrFloat64(outbuffer + offset, this->des_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->des_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->des_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_angvel_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_angvel_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_angvel_z);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_acc_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_acc_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->est_acc_z);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->pwm[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pwm[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pwm[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->quad_time =  ((uint16_t) (*(inbuffer + offset)));
      this->quad_time |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->quad_time);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->des_thrust));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->des_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->des_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->des_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_angvel_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_angvel_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_angvel_z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_acc_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_acc_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->est_acc_z));
      for( uint32_t i = 0; i < 4; i++){
      this->pwm[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->pwm[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pwm[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/PPROutputData"; };
    virtual const char * getMD5() override { return "732c0e3ca36f241464f8c445e78a0d0a"; };

  };

}
#endif
