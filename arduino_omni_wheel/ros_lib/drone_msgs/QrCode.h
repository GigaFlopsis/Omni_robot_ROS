#ifndef _ROS_drone_msgs_QrCode_h
#define _ROS_drone_msgs_QrCode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace drone_msgs
{

  class QrCode : public ros::Msg
  {
    public:
      typedef const char* _data_type;
      _data_type data;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;

    QrCode():
      data(""),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "drone_msgs/QrCode"; };
    virtual const char * getMD5() override { return "4f4bf8628b24695eaa28a1ce10b18688"; };

  };

}
#endif
