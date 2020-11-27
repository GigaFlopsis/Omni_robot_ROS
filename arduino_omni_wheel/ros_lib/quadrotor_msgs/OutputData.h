#ifndef _ROS_quadrotor_msgs_OutputData_h
#define _ROS_quadrotor_msgs_OutputData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

namespace quadrotor_msgs
{

  class OutputData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _loop_rate_type;
      _loop_rate_type loop_rate;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef float _pressure_dheight_type;
      _pressure_dheight_type pressure_dheight;
      typedef float _pressure_height_type;
      _pressure_height_type pressure_height;
      typedef geometry_msgs::Vector3 _magnetic_field_type;
      _magnetic_field_type magnetic_field;
      uint8_t radio_channel[8];
      typedef uint8_t _seq_type;
      _seq_type seq;

    OutputData():
      header(),
      loop_rate(0),
      voltage(0),
      orientation(),
      angular_velocity(),
      linear_acceleration(),
      pressure_dheight(0),
      pressure_height(0),
      magnetic_field(),
      radio_channel(),
      seq(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->loop_rate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loop_rate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->loop_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_dheight);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_height);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->radio_channel[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->radio_channel[i]);
      }
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      offset += sizeof(this->seq);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->loop_rate =  ((uint16_t) (*(inbuffer + offset)));
      this->loop_rate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->loop_rate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage));
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_dheight));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_height));
      offset += this->magnetic_field.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 8; i++){
      this->radio_channel[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->radio_channel[i]);
      }
      this->seq =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->seq);
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/OutputData"; };
    virtual const char * getMD5() override { return "5759ee97fd5c090dcdccf7fc3e50d024"; };

  };

}
#endif
