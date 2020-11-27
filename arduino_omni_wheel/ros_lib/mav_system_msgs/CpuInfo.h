#ifndef _ROS_mav_system_msgs_CpuInfo_h
#define _ROS_mav_system_msgs_CpuInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mav_system_msgs/ProcessInfo.h"

namespace mav_system_msgs
{

  class CpuInfo : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t cpu_percent_length;
      typedef float _cpu_percent_type;
      _cpu_percent_type st_cpu_percent;
      _cpu_percent_type * cpu_percent;
      uint32_t heaviest_processes_length;
      typedef mav_system_msgs::ProcessInfo _heaviest_processes_type;
      _heaviest_processes_type st_heaviest_processes;
      _heaviest_processes_type * heaviest_processes;

    CpuInfo():
      header(),
      cpu_percent_length(0), st_cpu_percent(), cpu_percent(nullptr),
      heaviest_processes_length(0), st_heaviest_processes(), heaviest_processes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->cpu_percent_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cpu_percent_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cpu_percent_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cpu_percent_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_percent_length);
      for( uint32_t i = 0; i < cpu_percent_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cpu_percenti;
      u_cpu_percenti.real = this->cpu_percent[i];
      *(outbuffer + offset + 0) = (u_cpu_percenti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpu_percenti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpu_percenti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpu_percenti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cpu_percent[i]);
      }
      *(outbuffer + offset + 0) = (this->heaviest_processes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->heaviest_processes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->heaviest_processes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->heaviest_processes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heaviest_processes_length);
      for( uint32_t i = 0; i < heaviest_processes_length; i++){
      offset += this->heaviest_processes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t cpu_percent_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cpu_percent_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cpu_percent_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cpu_percent_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cpu_percent_length);
      if(cpu_percent_lengthT > cpu_percent_length)
        this->cpu_percent = (float*)realloc(this->cpu_percent, cpu_percent_lengthT * sizeof(float));
      cpu_percent_length = cpu_percent_lengthT;
      for( uint32_t i = 0; i < cpu_percent_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cpu_percent;
      u_st_cpu_percent.base = 0;
      u_st_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cpu_percent.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cpu_percent = u_st_cpu_percent.real;
      offset += sizeof(this->st_cpu_percent);
        memcpy( &(this->cpu_percent[i]), &(this->st_cpu_percent), sizeof(float));
      }
      uint32_t heaviest_processes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      heaviest_processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      heaviest_processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      heaviest_processes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->heaviest_processes_length);
      if(heaviest_processes_lengthT > heaviest_processes_length)
        this->heaviest_processes = (mav_system_msgs::ProcessInfo*)realloc(this->heaviest_processes, heaviest_processes_lengthT * sizeof(mav_system_msgs::ProcessInfo));
      heaviest_processes_length = heaviest_processes_lengthT;
      for( uint32_t i = 0; i < heaviest_processes_length; i++){
      offset += this->st_heaviest_processes.deserialize(inbuffer + offset);
        memcpy( &(this->heaviest_processes[i]), &(this->st_heaviest_processes), sizeof(mav_system_msgs::ProcessInfo));
      }
     return offset;
    }

    virtual const char * getType() override { return "mav_system_msgs/CpuInfo"; };
    virtual const char * getMD5() override { return "d955ed6c182adaafbfa41e78710954d2"; };

  };

}
#endif
