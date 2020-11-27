#ifndef _ROS_nbv_msgs_State_h
#define _ROS_nbv_msgs_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nbv_msgs/Status.h"

namespace nbv_msgs
{

  class State : public ros::Msg
  {
    public:
      typedef nbv_msgs::Status _status_type;
      _status_type status;
      typedef const char* _data_type;
      _data_type data;

    State():
      status(),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    virtual const char * getType() override { return "nbv_msgs/State"; };
    virtual const char * getMD5() override { return "44111470c683a352960c4642f4bc2ca4"; };

  };

}
#endif
