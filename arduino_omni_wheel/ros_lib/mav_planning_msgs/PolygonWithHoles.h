#ifndef _ROS_mav_planning_msgs_PolygonWithHoles_h
#define _ROS_mav_planning_msgs_PolygonWithHoles_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mav_planning_msgs/Polygon2D.h"

namespace mav_planning_msgs
{

  class PolygonWithHoles : public ros::Msg
  {
    public:
      typedef mav_planning_msgs::Polygon2D _hull_type;
      _hull_type hull;
      uint32_t holes_length;
      typedef mav_planning_msgs::Polygon2D _holes_type;
      _holes_type st_holes;
      _holes_type * holes;

    PolygonWithHoles():
      hull(),
      holes_length(0), st_holes(), holes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->hull.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->holes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->holes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->holes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->holes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->holes_length);
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->holes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->hull.deserialize(inbuffer + offset);
      uint32_t holes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->holes_length);
      if(holes_lengthT > holes_length)
        this->holes = (mav_planning_msgs::Polygon2D*)realloc(this->holes, holes_lengthT * sizeof(mav_planning_msgs::Polygon2D));
      holes_length = holes_lengthT;
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->st_holes.deserialize(inbuffer + offset);
        memcpy( &(this->holes[i]), &(this->st_holes), sizeof(mav_planning_msgs::Polygon2D));
      }
     return offset;
    }

    virtual const char * getType() override { return "mav_planning_msgs/PolygonWithHoles"; };
    virtual const char * getMD5() override { return "df7f266352dfcf3e4d29156dd85febf9"; };

  };

}
#endif
