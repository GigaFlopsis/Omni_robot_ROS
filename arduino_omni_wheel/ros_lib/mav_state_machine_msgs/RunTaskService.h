#ifndef _ROS_SERVICE_RunTaskService_h
#define _ROS_SERVICE_RunTaskService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mav_state_machine_msgs
{

static const char RUNTASKSERVICE[] = "mav_state_machine_msgs/RunTaskService";

  class RunTaskServiceRequest : public ros::Msg
  {
    public:
      typedef const char* _task_name_type;
      _task_name_type task_name;
      typedef bool _start_type;
      _start_type start;

    RunTaskServiceRequest():
      task_name(""),
      start(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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

    virtual const char * getType() override { return RUNTASKSERVICE; };
    virtual const char * getMD5() override { return "78def82808ce5580c326beaff29b4920"; };

  };

  class RunTaskServiceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    RunTaskServiceResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return RUNTASKSERVICE; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class RunTaskService {
    public:
    typedef RunTaskServiceRequest Request;
    typedef RunTaskServiceResponse Response;
  };

}
#endif
