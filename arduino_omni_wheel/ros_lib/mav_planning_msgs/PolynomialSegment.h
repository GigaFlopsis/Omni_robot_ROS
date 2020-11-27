#ifndef _ROS_mav_planning_msgs_PolynomialSegment_h
#define _ROS_mav_planning_msgs_PolynomialSegment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace mav_planning_msgs
{

  class PolynomialSegment : public ros::Msg
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
      uint32_t rx_length;
      typedef float _rx_type;
      _rx_type st_rx;
      _rx_type * rx;
      uint32_t ry_length;
      typedef float _ry_type;
      _ry_type st_ry;
      _ry_type * ry;
      uint32_t rz_length;
      typedef float _rz_type;
      _rz_type st_rz;
      _rz_type * rz;
      uint32_t yaw_length;
      typedef float _yaw_type;
      _yaw_type st_yaw;
      _yaw_type * yaw;

    PolynomialSegment():
      header(),
      num_coeffs(0),
      segment_time(),
      x_length(0), st_x(), x(nullptr),
      y_length(0), st_y(), y(nullptr),
      z_length(0), st_z(), z(nullptr),
      rx_length(0), st_rx(), rx(nullptr),
      ry_length(0), st_ry(), ry(nullptr),
      rz_length(0), st_rz(), rz(nullptr),
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
      *(outbuffer + offset + 0) = (this->rx_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rx_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rx_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rx_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rx_length);
      for( uint32_t i = 0; i < rx_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rx[i]);
      }
      *(outbuffer + offset + 0) = (this->ry_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ry_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ry_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ry_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ry_length);
      for( uint32_t i = 0; i < ry_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ry[i]);
      }
      *(outbuffer + offset + 0) = (this->rz_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rz_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rz_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rz_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rz_length);
      for( uint32_t i = 0; i < rz_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rz[i]);
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
      uint32_t rx_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rx_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rx_length);
      if(rx_lengthT > rx_length)
        this->rx = (float*)realloc(this->rx, rx_lengthT * sizeof(float));
      rx_length = rx_lengthT;
      for( uint32_t i = 0; i < rx_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_rx));
        memcpy( &(this->rx[i]), &(this->st_rx), sizeof(float));
      }
      uint32_t ry_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ry_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ry_length);
      if(ry_lengthT > ry_length)
        this->ry = (float*)realloc(this->ry, ry_lengthT * sizeof(float));
      ry_length = ry_lengthT;
      for( uint32_t i = 0; i < ry_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_ry));
        memcpy( &(this->ry[i]), &(this->st_ry), sizeof(float));
      }
      uint32_t rz_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rz_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rz_length);
      if(rz_lengthT > rz_length)
        this->rz = (float*)realloc(this->rz, rz_lengthT * sizeof(float));
      rz_length = rz_lengthT;
      for( uint32_t i = 0; i < rz_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_rz));
        memcpy( &(this->rz[i]), &(this->st_rz), sizeof(float));
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

    virtual const char * getType() override { return "mav_planning_msgs/PolynomialSegment"; };
    virtual const char * getMD5() override { return "1bfc920140297f14773c46c1eacc4c1d"; };

  };

}
#endif
