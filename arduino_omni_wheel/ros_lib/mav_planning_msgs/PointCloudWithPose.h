#ifndef _ROS_mav_planning_msgs_PointCloudWithPose_h
#define _ROS_mav_planning_msgs_PointCloudWithPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/PointCloud2.h"

namespace mav_planning_msgs
{

  class PointCloudWithPose : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::TransformStamped _sensor_pose_type;
      _sensor_pose_type sensor_pose;
      typedef sensor_msgs::PointCloud2 _cloud_in_sensor_frame_type;
      _cloud_in_sensor_frame_type cloud_in_sensor_frame;

    PointCloudWithPose():
      header(),
      sensor_pose(),
      cloud_in_sensor_frame()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->sensor_pose.serialize(outbuffer + offset);
      offset += this->cloud_in_sensor_frame.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->sensor_pose.deserialize(inbuffer + offset);
      offset += this->cloud_in_sensor_frame.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mav_planning_msgs/PointCloudWithPose"; };
    virtual const char * getMD5() override { return "2a8b498d41262fbae6e2ab39e0965442"; };

  };

}
#endif
