#ifndef _ROS_mav_state_machine_msgs_StartStopTask_h
#define _ROS_mav_state_machine_msgs_StartStopTask_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mav_state_machine_msgs
{

  class StartStopTask : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _task_name_type;
      _task_name_type task_name;
      typedef bool _start_type;
      _start_type start;

    StartStopTask():
      header(),
      task_name(""),
      start(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_task_name = strlen(this->task_name);
      varToArr(outbuffer + offset, length_task_name);
      offset += 4;
      memcpy(outbuffer + offset, this->task_name, length_task_name);
      offset += length_task_name;
      union {
        bool real;
        uint8_t base;
      } u_start;
      u_start.real = this->start;
      *(outbuffer + offset + 0) = (u_start.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_task_name;
      arrToVar(length_task_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_task_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_task_name-1]=0;
      this->task_name = (char *)(inbuffer + offset-1);
      offset += length_task_name;
      union {
        bool real;
        uint8_t base;
      } u_start;
      u_start.base = 0;
      u_start.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start = u_start.real;
      offset += sizeof(this->start);
     return offset;
    }

    virtual const char * getType() override { return "mav_state_machine_msgs/StartStopTask"; };
    virtual const char * getMD5() override { return "10cd1c89cea1d199a1d9752c42bc712c"; };

  };

}
#endif
