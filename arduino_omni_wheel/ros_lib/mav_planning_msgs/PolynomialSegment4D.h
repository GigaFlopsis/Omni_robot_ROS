#ifndef _ROS_mav_planning_msgs_PolynomialSegment4D_h
#define _ROS_mav_planning_msgs_PolynomialSegment4D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace mav_planning_msgs
{

  class PolynomialSegment4D : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _num_coeffs_type;
      _num_coeffs_type num_coeffs;
      typedef ros::Duration _segment_time_type;
      _segment_time_type segment_time;
      uint32_t x_length;
      typedef float _x_type;
      _x_type st_x;
      _x_type * x;
      uint32_t y_length;
      typedef float _y_type;
      _y_type st_y;
      _y_type * y;
      uint32_t z_length;
      typedef float _z_type;
      _z_type st_z;
      _z_type * z;
      uint32_t yaw_length;
      typedef float _yaw_type;
      _yaw_type st_yaw;
      _yaw_type * yaw;

    PolynomialSegment4D():
      header(),
      num_coeffs(0),
      segment_time(),
      x_length(0), st_x(), x(nullptr),
      y_length(0), st_y(), y(nullptr),
      z_length(0), st_z(), z(nullptr),
      yaw_length(0), st_yaw(), yaw(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_num_coeffs;
      u_num_coeffs.real = this->num_coeffs;
      *(outbuffer + offset + 0) = (u_num_coeffs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_coeffs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_coeffs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_coeffs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_coeffs);
      *(outbuffer + offset + 0) = (this->segment_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segment_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segment_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segment_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_time.sec);
      *(outbuffer + offset + 0) = (this->segment_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segment_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segment_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segment_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_time.nsec);
      *(outbuffer + offset + 0) = (this->x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_length);
      for( uint32_t i = 0; i < x_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->x[i]);
      }
      *(outbuffer + offset + 0) = (this->y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_length);
      for( uint32_t i = 0; i < y_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->y[i]);
      }
      *(outbuffer + offset + 0) = (this->z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_length);
      for( uint32_t i = 0; i < z_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->z[i]);
      }
      *(outbuffer + offset + 0) = (this->yaw_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_length);
      for( uint32_t i = 0; i < yaw_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_num_coeffs;
      u_num_coeffs.base = 0;
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_coeffs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_coeffs = u_num_coeffs.real;
      offset += sizeof(this->num_coeffs);
      this->segment_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->segment_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->segment_time.sec);
      this->segment_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->segment_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->segment_time.nsec);
      uint32_t x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->x_length);
      if(x_lengthT > x_length)
        this->x = (float*)realloc(this->x, x_lengthT * sizeof(float));
      x_length = x_lengthT;
      for( uint32_t i = 0; i < x_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_x));
        memcpy( &(this->x[i]), &(this->st_x), sizeof(float));
      }
      uint32_t y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->y_length);
      if(y_lengthT > y_length)
        this->y = (float*)realloc(this->y, y_lengthT * sizeof(float));
      y_length = y_lengthT;
      for( uint32_t i = 0; i < y_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_y));
        memcpy( &(this->y[i]), &(this->st_y), sizeof(float));
      }
      uint32_t z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_length);
      if(z_lengthT > z_length)
        this->z = (float*)realloc(this->z, z_lengthT * sizeof(float));
      z_length = z_lengthT;
      for( uint32_t i = 0; i < z_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_z));
        memcpy( &(this->z[i]), &(this->st_z), sizeof(float));
      }
      uint32_t yaw_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_length);
      if(yaw_lengthT > yaw_length)
        this->yaw = (float*)realloc(this->yaw, yaw_lengthT * sizeof(float));
      yaw_length = yaw_lengthT;
      for( uint32_t i = 0; i < yaw_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_yaw));
        memcpy( &(this->yaw[i]), &(this->st_yaw), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "mav_planning_msgs/PolynomialSegment4D"; };
    virtual const char * getMD5() override { return "c85fa40c94ff35d242df13d4d3a57809"; };

  };

}
#endif
