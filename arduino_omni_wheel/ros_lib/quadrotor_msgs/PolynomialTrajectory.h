#ifndef _ROS_quadrotor_msgs_PolynomialTrajectory_h
#define _ROS_quadrotor_msgs_PolynomialTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrotor_msgs
{

  class PolynomialTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef uint32_t _action_type;
      _action_type action;
      typedef uint32_t _num_order_type;
      _num_order_type num_order;
      typedef uint32_t _num_segment_type;
      _num_segment_type num_segment;
      typedef float _start_yaw_type;
      _start_yaw_type start_yaw;
      typedef float _final_yaw_type;
      _final_yaw_type final_yaw;
      uint32_t coef_x_length;
      typedef float _coef_x_type;
      _coef_x_type st_coef_x;
      _coef_x_type * coef_x;
      uint32_t coef_y_length;
      typedef float _coef_y_type;
      _coef_y_type st_coef_y;
      _coef_y_type * coef_y;
      uint32_t coef_z_length;
      typedef float _coef_z_type;
      _coef_z_type st_coef_z;
      _coef_z_type * coef_z;
      uint32_t time_length;
      typedef float _time_type;
      _time_type st_time;
      _time_type * time;
      typedef float _mag_coeff_type;
      _mag_coeff_type mag_coeff;
      uint32_t order_length;
      typedef uint32_t _order_type;
      _order_type st_order;
      _order_type * order;
      typedef const char* _debug_info_type;
      _debug_info_type debug_info;
      enum { ACTION_ADD =    1 };
      enum { ACTION_ABORT =    2 };
      enum { ACTION_WARN_START =    3 };
      enum { ACTION_WARN_FINAL =    4 };
      enum { ACTION_WARN_IMPOSSIBLE =    5 };

    PolynomialTrajectory():
      header(),
      trajectory_id(0),
      action(0),
      num_order(0),
      num_segment(0),
      start_yaw(0),
      final_yaw(0),
      coef_x_length(0), st_coef_x(), coef_x(nullptr),
      coef_y_length(0), st_coef_y(), coef_y(nullptr),
      coef_z_length(0), st_coef_z(), coef_z(nullptr),
      time_length(0), st_time(), time(nullptr),
      mag_coeff(0),
      order_length(0), st_order(), order(nullptr),
      debug_info("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trajectory_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id);
      *(outbuffer + offset + 0) = (this->action >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->action >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->action >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->action >> (8 * 3)) & 0xFF;
      offset += sizeof(this->action);
      *(outbuffer + offset + 0) = (this->num_order >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_order >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_order >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_order >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_order);
      *(outbuffer + offset + 0) = (this->num_segment >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_segment >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_segment >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_segment >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_segment);
      offset += serializeAvrFloat64(outbuffer + offset, this->start_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->final_yaw);
      *(outbuffer + offset + 0) = (this->coef_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coef_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coef_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coef_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coef_x_length);
      for( uint32_t i = 0; i < coef_x_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->coef_x[i]);
      }
      *(outbuffer + offset + 0) = (this->coef_y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coef_y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coef_y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coef_y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coef_y_length);
      for( uint32_t i = 0; i < coef_y_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->coef_y[i]);
      }
      *(outbuffer + offset + 0) = (this->coef_z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->coef_z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->coef_z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->coef_z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->coef_z_length);
      for( uint32_t i = 0; i < coef_z_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->coef_z[i]);
      }
      *(outbuffer + offset + 0) = (this->time_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_length);
      for( uint32_t i = 0; i < time_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->time[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->mag_coeff);
      *(outbuffer + offset + 0) = (this->order_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->order_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->order_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->order_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->order_length);
      for( uint32_t i = 0; i < order_length; i++){
      *(outbuffer + offset + 0) = (this->order[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->order[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->order[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->order[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->order[i]);
      }
      uint32_t length_debug_info = strlen(this->debug_info);
      varToArr(outbuffer + offset, length_debug_info);
      offset += 4;
      memcpy(outbuffer + offset, this->debug_info, length_debug_info);
      offset += length_debug_info;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->trajectory_id =  ((uint32_t) (*(inbuffer + offset)));
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->trajectory_id);
      this->action =  ((uint32_t) (*(inbuffer + offset)));
      this->action |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->action |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->action |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->action);
      this->num_order =  ((uint32_t) (*(inbuffer + offset)));
      this->num_order |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_order |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_order |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_order);
      this->num_segment =  ((uint32_t) (*(inbuffer + offset)));
      this->num_segment |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_segment |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_segment |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_segment);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->start_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->final_yaw));
      uint32_t coef_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coef_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coef_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coef_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coef_x_length);
      if(coef_x_lengthT > coef_x_length)
        this->coef_x = (float*)realloc(this->coef_x, coef_x_lengthT * sizeof(float));
      coef_x_length = coef_x_lengthT;
      for( uint32_t i = 0; i < coef_x_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_coef_x));
        memcpy( &(this->coef_x[i]), &(this->st_coef_x), sizeof(float));
      }
      uint32_t coef_y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coef_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coef_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coef_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coef_y_length);
      if(coef_y_lengthT > coef_y_length)
        this->coef_y = (float*)realloc(this->coef_y, coef_y_lengthT * sizeof(float));
      coef_y_length = coef_y_lengthT;
      for( uint32_t i = 0; i < coef_y_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_coef_y));
        memcpy( &(this->coef_y[i]), &(this->st_coef_y), sizeof(float));
      }
      uint32_t coef_z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      coef_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      coef_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      coef_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->coef_z_length);
      if(coef_z_lengthT > coef_z_length)
        this->coef_z = (float*)realloc(this->coef_z, coef_z_lengthT * sizeof(float));
      coef_z_length = coef_z_lengthT;
      for( uint32_t i = 0; i < coef_z_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_coef_z));
        memcpy( &(this->coef_z[i]), &(this->st_coef_z), sizeof(float));
      }
      uint32_t time_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      time_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->time_length);
      if(time_lengthT > time_length)
        this->time = (float*)realloc(this->time, time_lengthT * sizeof(float));
      time_length = time_lengthT;
      for( uint32_t i = 0; i < time_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_time));
        memcpy( &(this->time[i]), &(this->st_time), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mag_coeff));
      uint32_t order_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      order_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      order_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      order_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->order_length);
      if(order_lengthT > order_length)
        this->order = (uint32_t*)realloc(this->order, order_lengthT * sizeof(uint32_t));
      order_length = order_lengthT;
      for( uint32_t i = 0; i < order_length; i++){
      this->st_order =  ((uint32_t) (*(inbuffer + offset)));
      this->st_order |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_order |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_order |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_order);
        memcpy( &(this->order[i]), &(this->st_order), sizeof(uint32_t));
      }
      uint32_t length_debug_info;
      arrToVar(length_debug_info, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_debug_info; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_debug_info-1]=0;
      this->debug_info = (char *)(inbuffer + offset-1);
      offset += length_debug_info;
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/PolynomialTrajectory"; };
    virtual const char * getMD5() override { return "3abb6e1147f95babc52b64612c5ba5ed"; };

  };

}
#endif
