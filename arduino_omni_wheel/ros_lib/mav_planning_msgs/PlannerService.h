#ifndef _ROS_SERVICE_PlannerService_h
#define _ROS_SERVICE_PlannerService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "mav_planning_msgs/PolynomialTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "geometry_msgs/Vector3.h"
#include "mav_planning_msgs/PolynomialTrajectory4D.h"

namespace mav_planning_msgs
{

static const char PLANNERSERVICE[] = "mav_planning_msgs/PlannerService";

  class PlannerServiceRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _start_pose_type;
      _start_pose_type start_pose;
      typedef geometry_msgs::Vector3 _start_velocity_type;
      _start_velocity_type start_velocity;
      typedef geometry_msgs::PoseStamped _goal_pose_type;
      _goal_pose_type goal_pose;
      typedef geometry_msgs::Vector3 _goal_velocity_type;
      _goal_velocity_type goal_velocity;
      typedef geometry_msgs::Vector3 _bounding_box_type;
      _bounding_box_type bounding_box;

    PlannerServiceRequest():
      start_pose(),
      start_velocity(),
      goal_pose(),
      goal_velocity(),
      bounding_box()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->start_pose.serialize(outbuffer + offset);
      offset += this->start_velocity.serialize(outbuffer + offset);
      offset += this->goal_pose.serialize(outbuffer + offset);
      offset += this->goal_velocity.serialize(outbuffer + offset);
      offset += this->bounding_box.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->start_pose.deserialize(inbuffer + offset);
      offset += this->start_velocity.deserialize(inbuffer + offset);
      offset += this->goal_pose.deserialize(inbuffer + offset);
      offset += this->goal_velocity.deserialize(inbuffer + offset);
      offset += this->bounding_box.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return PLANNERSERVICE; };
    virtual const char * getMD5() override { return "6090fe8ab3df1362b8c26859b8850940"; };

  };

  class PlannerServiceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef mav_planning_msgs::PolynomialTrajectory _polynomial_plan_type;
      _polynomial_plan_type polynomial_plan;
      typedef mav_planning_msgs::PolynomialTrajectory4D _polynomial_plan_4D_type;
      _polynomial_plan_4D_type polynomial_plan_4D;
      typedef trajectory_msgs::MultiDOFJointTrajectory _sampled_plan_type;
      _sampled_plan_type sampled_plan;

    PlannerServiceResponse():
      success(0),
      polynomial_plan(),
      polynomial_plan_4D(),
      sampled_plan()
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
      offset += this->polynomial_plan.serialize(outbuffer + offset);
      offset += this->polynomial_plan_4D.serialize(outbuffer + offset);
      offset += this->sampled_plan.serialize(outbuffer + offset);
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
      offset += this->polynomial_plan.deserialize(inbuffer + offset);
      offset += this->polynomial_plan_4D.deserialize(inbuffer + offset);
      offset += this->sampled_plan.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return PLANNERSERVICE; };
    virtual const char * getMD5() override { return "2dfc73ed0482f368aa016a76f49c8aed"; };

  };

  class PlannerService {
    public:
    typedef PlannerServiceRequest Request;
    typedef PlannerServiceResponse Response;
  };

}
#endif
