#ifndef _ROS_mav_planning_msgs_PolynomialTrajectory4D_h
#define _ROS_mav_planning_msgs_PolynomialTrajectory4D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mav_planning_msgs/PolynomialSegment4D.h"

namespace mav_planning_msgs
{

  class PolynomialTrajectory4D : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t segments_length;
      typedef mav_planning_msgs::PolynomialSegment4D _segments_type;
      _segments_type st_segments;
      _segments_type * segments;

    PolynomialTrajectory4D():
      header(),
      segments_length(0), st_segments(), segments(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->segments_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->segments_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->segments_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->segments_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segments_length);
      for( uint32_t i = 0; i < segments_length; i++){
      offset += this->segments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t segments_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      segments_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->segments_length);
      if(segments_lengthT > segments_length)
        this->segments = (mav_planning_msgs::PolynomialSegment4D*)realloc(this->segments, segments_lengthT * sizeof(mav_planning_msgs::PolynomialSegment4D));
      segments_length = segments_lengthT;
      for( uint32_t i = 0; i < segments_length; i++){
      offset += this->st_segments.deserialize(inbuffer + offset);
        memcpy( &(this->segments[i]), &(this->st_segments), sizeof(mav_planning_msgs::PolynomialSegment4D));
      }
     return offset;
    }

    virtual const char * getType() override { return "mav_planning_msgs/PolynomialTrajectory4D"; };
    virtual const char * getMD5() override { return "4d68d15524ede489eecd674bb6dc3ee8"; };

  };

}
#endif
