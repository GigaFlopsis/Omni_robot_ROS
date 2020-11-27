#ifndef _ROS_mav_system_msgs_ProcessInfo_h
#define _ROS_mav_system_msgs_ProcessInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mav_system_msgs
{

  class ProcessInfo : public ros::Msg
  {
    public:
      typedef uint32_t _pid_type;
      _pid_type pid;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _username_type;
      _username_type username;
      typedef float _cpu_percent_type;
      _cpu_percent_type cpu_percent;

    ProcessInfo():
      pid(0),
      name(""),
      username(""),
      cpu_percent(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pid >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pid >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pid >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pid);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_username = strlen(this->username);
      varToArr(outbuffer + offset, length_username);
      offset += 4;
      memcpy(outbuffer + offset, this->username, length_username);
      offset += length_username;
      union {
        float real;
        uint32_t base;
      } u_cpu_percent;
      u_cpu_percent.real = this->cpu_percent;
      *(outbuffer + offset + 0) = (u_cpu_percent.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_percent.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_percent.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_percent.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_percent);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->pid =  ((uint32_t) (*(inbuffer + offset)));
      this->pid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pid);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_username;
      arrToVar(length_username, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_username; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_username-1]=0;
      this->username = (char *)(inbuffer + offset-1);
      offset += length_username;
      union {
        float real;
        uint32_t base;
      } u_cpu_percent;
      u_cpu_percent.base = 0;
      u_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cpu_percent = u_cpu_percent.real;
      offset += sizeof(this->cpu_percent);
     return offset;
    }

    virtual const char * getType() override { return "mav_system_msgs/ProcessInfo"; };
    virtual const char * getMD5() override { return "e230da576ecad86012b88749ce2ed125"; };

  };

}
#endif
