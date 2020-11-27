#ifndef _ROS_quadrotor_msgs_LQRTrajectory_h
#define _ROS_quadrotor_msgs_LQRTrajectory_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrotor_msgs
{

  class LQRTrajectory : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef uint32_t _action_type;
      _action_type action;
      typedef float _r_type;
      _r_type r;
      typedef float _start_yaw_type;
      _start_yaw_type start_yaw;
      typedef float _final_yaw_type;
      _final_yaw_type final_yaw;
      float s0[6];
      float ut[3];
      float sf[6];
      typedef float _t_f_type;
      _t_f_type t_f;
      typedef const char* _debug_info_type;
      _debug_info_type debug_info;
      enum { ACTION_ADD =    1 };
      enum { ACTION_ABORT =    2 };
      enum { ACTION_WARN_START =    3 };
      enum { ACTION_WARN_FINAL =    4 };
      enum { ACTION_WARN_IMPOSSIBLE =    5 };

    LQRTrajectory():
      header(),
      trajectory_id(0),
      action(0),
      r(0),
      start_yaw(0),
      final_yaw(0),
      s0(),
      ut(),
      sf(),
      t_f(0),
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
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      offset += serializeAvrFloat64(outbuffer + offset, this->start_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->final_yaw);
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->s0[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->ut[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->sf[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->t_f);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->start_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->final_yaw));
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->s0[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ut[i]));
      }
      for( uint32_t i = 0; i < 6; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sf[i]));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->t_f));
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

    virtual const char * getType() override { return "quadrotor_msgs/LQRTrajectory"; };
    virtual const char * getMD5() override { return "46be446c56bc8bf131978edfc4464480"; };

  };

}
#endif
