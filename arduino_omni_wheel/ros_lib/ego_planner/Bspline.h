#ifndef _ROS_ego_planner_Bspline_h
#define _ROS_ego_planner_Bspline_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Point.h"

namespace ego_planner
{

  class Bspline : public ros::Msg
  {
    public:
      typedef int32_t _order_type;
      _order_type order;
      typedef int64_t _traj_id_type;
      _traj_id_type traj_id;
      typedef ros::Time _start_time_type;
      _start_time_type start_time;
      uint32_t knots_length;
      typedef float _knots_type;
      _knots_type st_knots;
      _knots_type * knots;
      uint32_t pos_pts_length;
      typedef geometry_msgs::Point _pos_pts_type;
      _pos_pts_type st_pos_pts;
      _pos_pts_type * pos_pts;
      uint32_t yaw_pts_length;
      typedef float _yaw_pts_type;
      _yaw_pts_type st_yaw_pts;
      _yaw_pts_type * yaw_pts;
      typedef float _yaw_dt_type;
      _yaw_dt_type yaw_dt;

    Bspline():
      order(0),
      traj_id(0),
      start_time(),
      knots_length(0), st_knots(), knots(nullptr),
      pos_pts_length(0), st_pos_pts(), pos_pts(nullptr),
      yaw_pts_length(0), st_yaw_pts(), yaw_pts(nullptr),
      yaw_dt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_order;
      u_order.real = this->order;
      *(outbuffer + offset + 0) = (u_order.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_order.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_order.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_order.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->order);
      union {
        int64_t real;
        uint64_t base;
      } u_traj_id;
      u_traj_id.real = this->traj_id;
      *(outbuffer + offset + 0) = (u_traj_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_traj_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_traj_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_traj_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_traj_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_traj_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_traj_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_traj_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->traj_id);
      *(outbuffer + offset + 0) = (this->start_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.sec);
      *(outbuffer + offset + 0) = (this->start_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.nsec);
      *(outbuffer + offset + 0) = (this->knots_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->knots_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->knots_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->knots_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->knots_length);
      for( uint32_t i = 0; i < knots_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->knots[i]);
      }
      *(outbuffer + offset + 0) = (this->pos_pts_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pos_pts_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pos_pts_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pos_pts_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pos_pts_length);
      for( uint32_t i = 0; i < pos_pts_length; i++){
      offset += this->pos_pts[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->yaw_pts_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_pts_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_pts_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_pts_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_pts_length);
      for( uint32_t i = 0; i < yaw_pts_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_pts[i]);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw_dt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_order;
      u_order.base = 0;
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_order.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->order = u_order.real;
      offset += sizeof(this->order);
      union {
        int64_t real;
        uint64_t base;
      } u_traj_id;
      u_traj_id.base = 0;
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_traj_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->traj_id = u_traj_id.real;
      offset += sizeof(this->traj_id);
      this->start_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.sec);
      this->start_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.nsec);
      uint32_t knots_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      knots_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      knots_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      knots_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->knots_length);
      if(knots_lengthT > knots_length)
        this->knots = (float*)realloc(this->knots, knots_lengthT * sizeof(float));
      knots_length = knots_lengthT;
      for( uint32_t i = 0; i < knots_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_knots));
        memcpy( &(this->knots[i]), &(this->st_knots), sizeof(float));
      }
      uint32_t pos_pts_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pos_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pos_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pos_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pos_pts_length);
      if(pos_pts_lengthT > pos_pts_length)
        this->pos_pts = (geometry_msgs::Point*)realloc(this->pos_pts, pos_pts_lengthT * sizeof(geometry_msgs::Point));
      pos_pts_length = pos_pts_lengthT;
      for( uint32_t i = 0; i < pos_pts_length; i++){
      offset += this->st_pos_pts.deserialize(inbuffer + offset);
        memcpy( &(this->pos_pts[i]), &(this->st_pos_pts), sizeof(geometry_msgs::Point));
      }
      uint32_t yaw_pts_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_pts_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_pts_length);
      if(yaw_pts_lengthT > yaw_pts_length)
        this->yaw_pts = (float*)realloc(this->yaw_pts, yaw_pts_lengthT * sizeof(float));
      yaw_pts_length = yaw_pts_lengthT;
      for( uint32_t i = 0; i < yaw_pts_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_yaw_pts));
        memcpy( &(this->yaw_pts[i]), &(this->st_yaw_pts), sizeof(float));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw_dt));
     return offset;
    }

    virtual const char * getType() override { return "ego_planner/Bspline"; };
    virtual const char * getMD5() override { return "b352d4f7278a546180de67cbe6793e49"; };

  };

}
#endif
