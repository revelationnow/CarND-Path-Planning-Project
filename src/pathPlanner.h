#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

#include <iostream>
#include <vector>
#include "util.h"
#include "spline.h"
#include "BehaviorPlanner.h"

using namespace std;



class PathPlanner
{
  private:
    vector<double> _previous_path_x;
    vector<double> _previous_path_y;
    vector<double> _prev_speed;
    vector<double> _prev_acc;
    vector<double> _car_data;
    double _last_acc;
    vector< vector<double> > _map_data;
    vector< vector<double> > _sensor_data;
    double _max_speed;
    double _max_acc;
    double _max_jerk;
    double _max_d_step;
    double _max_s;

    double _caution_zone;
    double _danger_zone;

    int _max_wayp;
    double _lane_width;

    double _ts; /*!< Time step */

  public:
    PathPlanner()
    {
      OUTPUT_MSG(INFO,"Created PathPlanner Object");
    }

    ~PathPlanner()
    {
      OUTPUT_MSG(INFO,"Destroyed PathPlannerObject");
    }

    void SetPrevPath(vector<double> previous_path_x, vector<double> previous_path_y)
    {
      _previous_path_x = previous_path_x;
      _previous_path_y = previous_path_y;
      OUTPUT_MSG(INFO, "Set the Previous paths");
    }

    void SetCarData(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed)
    {
      _car_data.resize(MAX_CARDATA);
      _car_data[X_POS] = car_x;
      _car_data[Y_POS] = car_y;
      _car_data[S_POS] = car_s;
      _car_data[D_POS] = car_d;
      _car_data[YAW] = deg2rad(car_yaw);
      _car_data[SPEED] = car_speed;
      OUTPUT_MSG(INFO,"Set the car data : X : "<<_car_data[X_POS]<<
                       ", Y : "<<_car_data[Y_POS]<<
                       ", S : "<<_car_data[S_POS]<<
                       ", D : "<<_car_data[D_POS]<<
                       ", YAW : "<<_car_data[YAW]<<
                       ", Speed : "<<_car_data[SPEED]
                       );

    }

    void SetZoneLimits(double caution_zone, double danger_zone)
    {
      _caution_zone = caution_zone;
      _danger_zone = danger_zone;
      OUTPUT_MSG(INFO, "Zone limits Set : caution zone : "<<_caution_zone<<", Danger zone : "<<_danger_zone);
    }

    void SetMapData( vector<double> way_x, vector<double> way_y,
        vector<double> way_s, vector<double> way_dx, vector<double> way_dy)
    {
      _map_data.resize(MAX_MAPDATA);
      _map_data[X_WAY] = way_x;
      _map_data[Y_WAY] = way_y;
      _map_data[S_WAY] = way_s;
      _map_data[DX_WAY] = way_dx;
      _map_data[DY_WAY] = way_dy;
      OUTPUT_MSG(INFO,"Set the map data");

    }

    void SetMaxLimits(
        double max_speed,
        double max_acc,
        double max_jerk,
        int num_waypoints,
        double time_step,
        double max_s,
        double lane_width
        )
    {
      _max_speed = max_speed;
      _max_acc = max_acc;
      _max_jerk = max_jerk;
      _max_wayp = num_waypoints;
      _ts = time_step;
      _max_d_step = _max_speed * _ts;
      _max_s = max_s;
      _lane_width = lane_width;

      _prev_speed.resize(_max_wayp);
      _prev_acc.resize(_max_wayp);

      for(int i = 0; i < _max_wayp; i++)
      {
        _prev_speed[i] = 0;
        _prev_acc[i] = 0;
      }

      OUTPUT_MSG(INFO,"Set the max limits : Max speed="<<_max_speed<<
                      ", Max Acc="<<_max_acc<<
                      ", Max Jerk="<<_max_jerk<<
                      ", Max num waypoints="<<_max_wayp<<
                      ", time_step="<<_ts<<
                      ", Max s="<<_max_s<<
                      ", Lane width="<<_lane_width
                      );
    }

    double LimitS(double s)
    {
      while(s > _max_s)
      {
        s = s - _max_s;
      }
      return s;
    }

    double DToLaneNum(double d)
    {
      return (double)((int)d/4);
    }


    void PlanPath(
        vector<double> &next_x,
        vector<double> &next_y,
        vector<double> prev_x,
        vector<double> prev_y,
        vector< vector<double> > sensor_data
        );

    void UpdateBehaviour();

    void GetBehaviourUpdate();

    void BuildSpline(
        vector<double> prev_x,
        vector<double> prev_y,
        double lane,
        tk::spline &s_out,
        double &ref_x,
        double &ref_y,
        double &ref_yaw
        );

    void GetNextPointVel(double &cur_point_vel, double &cur_point_acc, double prev_point_vel, double prev_point_acc, double ref_vel)
    {
      double max_acc_to_use = _max_acc * ((ref_vel - prev_point_vel)/ref_vel);
      double max_jerk_to_use = _max_jerk * ((ref_vel - prev_point_vel)/ref_vel);
      if(prev_point_vel < ref_vel)
      {
        cur_point_acc = min(prev_point_acc + max_jerk_to_use * _ts, max_acc_to_use);
        cur_point_vel = min(prev_point_vel + prev_point_acc * _ts + (0.5 * max_jerk_to_use * _ts * _ts), _max_speed);
      }
      else
      {
        cur_point_acc = max(prev_point_acc - max_jerk_to_use * _ts, -max_acc_to_use);
        cur_point_vel = max(prev_point_vel + prev_point_acc * _ts - (0.5 * max_jerk_to_use * _ts * _ts), -_max_speed);
      }
    }

    int PredictedLane(vector<double> sensor_data)
    {
      return (int)(sensor_data[SENS_D]/_lane_width);
    }

    double SetSplineLaneByDanger(double target_lane, double curr_lane, vector< vector<double> > sensor_data)
    {
      if(target_lane == curr_lane)
        return curr_lane;

      for(int i = 0; i < sensor_data.size(); i++)
      {
        if(PredictedLane(sensor_data[i]) == (int)target_lane)
        {
          double distance = fabs(_car_data[S_POS] - sensor_data[i][SENS_S]);
          if(distance < _caution_zone)
          {
            return curr_lane;
          }
        }
      }
      return target_lane;
    }

    double LaneToD(double lane)
    {
      return (_lane_width/2) + (_lane_width * lane);
    }

};

void
PathPlanner::BuildSpline
(
 vector<double> prev_x,
 vector<double> prev_y,
 double lane,
 tk::spline &s_out,
 double &ref_x,
 double &ref_y,
 double &ref_yaw
)
{
  /* Pick 3 way points in the distance */
  vector<double> wp0 = getXY(_car_data[S_POS] + 30, LaneToD(lane), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);
  vector<double> wp1 = getXY(_car_data[S_POS] + 60, LaneToD(lane), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);
  vector<double> wp2 = getXY(_car_data[S_POS] + 90, LaneToD(lane), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);

  /* Create a spline for these three way points */
  vector<double> ptsx, ptsy;
  ref_yaw = _car_data[YAW];

  if(prev_x.size() < 2)
  {
    ptsx.push_back(_car_data[X_POS] - cos(_car_data[YAW]) );
    ptsy.push_back(_car_data[Y_POS] - sin(_car_data[YAW]) );

    ptsx.push_back(_car_data[X_POS]);
    ptsy.push_back(_car_data[Y_POS]);
  }
  else
  {
    ptsx.push_back(prev_x[prev_x.size()-2]);
    ptsx.push_back(prev_x[prev_x.size()-1]);
    ptsy.push_back(prev_y[prev_y.size()-2]);
    ptsy.push_back(prev_y[prev_y.size()-1]);
    ref_yaw = atan2( ( prev_y[prev_y.size() - 1] - prev_y[prev_y.size() - 2]),
                     ( prev_x[prev_x.size() - 1] - prev_x[prev_x.size() - 2])
                   );
  }

  ptsx.push_back(wp0[0]);
  ptsx.push_back(wp1[0]);
  ptsx.push_back(wp2[0]);

  ptsy.push_back(wp0[1]);
  ptsy.push_back(wp1[1]);
  ptsy.push_back(wp2[1]);

  ref_x = ptsx[1];
  ref_y = ptsy[1];

  for(int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] =  (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] =  (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
  }

  s_out.set_points(ptsx, ptsy);
}



void
PathPlanner::PlanPath
(
 vector<double> &next_x,
 vector<double> &next_y,
 vector<double> prev_x,
 vector<double> prev_y,
 vector< vector<double> > sensor_data
)
{
  /* Append points from previous waypoints */
  for(int i = 0; i < prev_x.size(); i++)
  {
    next_x.push_back(prev_x[i]);
    next_y.push_back(prev_y[i]);
    _prev_speed[i] = _prev_speed[_prev_speed.size() - prev_x.size() + i];
    _prev_acc[i] = _prev_acc[_prev_acc.size() - prev_x.size() + i];
  }

  /* Update behaviour planner with speed in previous points*/


  /* Get a reference velocity and lane command from behaviour planner */
  double ref_vel = _max_speed;
  double point_vel = _prev_speed[prev_x.size()];
  double point_acc = _prev_acc[prev_x.size()];
  double curr_lane    = DToLaneNum(_car_data[D_POS]);
  double target_lane  = curr_lane;
  double ref_x;
  double ref_y;
  double ref_yaw;
  tk::spline s;

  /* If target lane is not dangerous, build spline to target lane, else to current lane */
  double spline_lane = SetSplineLaneByDanger(target_lane, curr_lane, sensor_data);



  BuildSpline(prev_x, prev_y, spline_lane, s, ref_x, ref_y, ref_yaw);

  /* Based on reference velocity pick points along the spline as inputs */
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y * target_y));

  double x_add_on = 0;

  OUTPUT_MSG(DEBUG, "Point vel = "<<point_vel<<", Point acc = "<<point_acc);
  for(int i = 1; i <= 50 - prev_x.size(); i++)
  {
    /* Check sensor fusion data to see if any cars are nearby */


    /* Apply equations of motion for max acceleration without violating jerk */
    GetNextPointVel(point_vel, point_acc, point_vel, point_acc, ref_vel);
    OUTPUT_MSG(DEBUG, "Point vel = "<<point_vel<<", Point acc = "<<point_acc);
    double N = (target_dist/(_ts * point_vel));
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_point_temp = x_point;
    double y_point_temp = y_point;

    x_point = (x_point_temp * cos(ref_yaw) - y_point_temp * sin(ref_yaw)) + ref_x;
    y_point = (x_point_temp * sin(ref_yaw) + y_point_temp * cos(ref_yaw)) + ref_y;

    next_x.push_back(x_point);
    next_y.push_back(y_point);

    _last_acc = point_acc;
    _prev_speed[prev_x.size() + i] = point_vel;
    _prev_acc[prev_x.size() + i] = point_acc;

  }




}











#endif //_PATHPLANNER_H_
