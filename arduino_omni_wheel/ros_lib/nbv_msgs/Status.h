#ifndef _ROS_nbv_msgs_Status_h
#define _ROS_nbv_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace nbv_msgs
{

  class Status : public ros::Msg
  {
    public:
      typedef int8_t _status_type;
      _status_type status;
      enum { STATUS_WAIT =   0        	 };
      enum { STATUS_RUN =   1        	 };
      enum { STATUS_RESET =  2 		 };
      enum { STATUS_STOP =  3       	 };
      enum { STATUS_COMPLETE =  4         };
      enum { STATUS_GO_HOME =  5        	 };
      enum { STATUS_GO_HOME_COMPLETE =  6     };

    Status():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "nbv_msgs/Status"; };
    virtual const char * getMD5() override { return "70d8fd58922471951da049416c6af8e4"; };

  };

}
#endif
