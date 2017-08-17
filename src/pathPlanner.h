#ifndef _PATHPLANNER_H_
#define _PATHPLANNER_H_

/*======================================================================================================================
                                                    HEADER FILES
======================================================================================================================*/
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include "util.h"
#include "spline.h"
#include "BehaviorPlanner.h"

/*======================================================================================================================
                                                     NAMESPACES
======================================================================================================================*/
using namespace std;


/*======================================================================================================================
                                                  CLASS DECLARATION
======================================================================================================================*/

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

    BehaviorPlanner *_B;

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

      _B = new BehaviorPlanner(100, max_speed, LANE_MID, _lane_width);

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
        LanesEnum cur_lane,
        LanesEnum targ_lane,
        tk::spline &s_out,
        double &ref_x,
        double &ref_y,
        double &ref_yaw
        );

    void GetNextPointVel(double &cur_point_vel, double &cur_point_acc, double prev_point_vel, double prev_point_acc, double ref_vel, double min_dist)
    {
      double max_acc_to_use = _max_acc * fabs((ref_vel - prev_point_vel)/ref_vel);

      if(min_dist < 0.5 * _danger_zone)
        max_acc_to_use = _max_acc;

      double max_jerk_to_use = _max_jerk;// * fabs((ref_vel - prev_point_vel)/ref_vel);
      if(prev_point_vel < ref_vel)
      {
        cur_point_acc = min(prev_point_acc + max_jerk_to_use * _ts, max_acc_to_use);
        cur_point_vel = min(min(prev_point_vel + prev_point_acc * _ts + (0.5 * max_jerk_to_use * _ts * _ts), _max_speed),ref_vel);
      }
      else
      {
        cur_point_acc = max(prev_point_acc - max_jerk_to_use * _ts, -max_acc_to_use);
        cur_point_vel = max(max(prev_point_vel + prev_point_acc * _ts - (0.5 * max_jerk_to_use * _ts * _ts), -_max_speed),-ref_vel);
      }

      if(cur_point_vel > 0)
      {
        cur_point_vel = min(cur_point_vel,_max_speed);
      }
      else
      {
        cur_point_vel = max(cur_point_vel, -_max_speed);
      }

    }

    bool PredictedLane(vector<double> sensor_data, LanesEnum lane)
    {
      double car_d = LaneToD(lane, _lane_width);
      if(fabs(car_d - sensor_data[SENS_D]) < 0.8 * _lane_width )
        return true;
      return false;

//      return (DToLane(sensor_data[SENS_D],_lane_width));
    }

    LanesEnum SetSplineLaneByDanger(LanesEnum target_lane, LanesEnum curr_lane, vector< vector<double> > sensor_data)
    {
      if(target_lane == curr_lane)
        return curr_lane;

      for(int i = 0; i < sensor_data.size(); i++)
      {
        if(true == PredictedLane(sensor_data[i], target_lane))
        {
          double distance = fabs(_car_data[S_POS] - sensor_data[i][SENS_S]);
          if(distance < _danger_zone)
          {
            return curr_lane;
          }
        }
      }
      return target_lane;
    }

    double GetRefVelBasedOnSensor(double original_ref_vel, LanesEnum lane, vector< vector<double> > sensor_data, double &min_dist)
    {
      for(int i = 0; i < sensor_data.size(); i++)
      {
        if((true  == PredictedLane(sensor_data[i], lane)) && (_car_data[S_POS] < sensor_data[i][SENS_S]))
        {
          double distance = fabs(_car_data[S_POS] - sensor_data[i][SENS_S]);
          double sensor_car_s_speed = sqrt((sensor_data[i][SENS_VX] * sensor_data[i][SENS_VX]) + (sensor_data[i][SENS_VY] * sensor_data[i][SENS_VY]));
          if(distance < min_dist)
          {
            min_dist = distance;
          }
          if(distance < _danger_zone)
          {
            OUTPUT_MSG(DEBUG, "!!!!! In Danger Zone : car_s : "<<_car_data[S_POS]<<", other_car_s : "<<sensor_data[i][SENS_S]);
            OUTPUT_MSG(DEBUG, "Original Ref Vel : "<<original_ref_vel<<", sensor_car_s_speed * 0.7 : "<<sensor_car_s_speed * 0.7);
            return min(original_ref_vel, sensor_car_s_speed * 0.7);
          }
          if(distance < _caution_zone)
          {
            //OUTPUT_MSG(DEBUG, "!! In Caution Zone : car_s : "<<_car_data[S_POS]<<", other_car_s : "<<sensor_data[i][SENS_S]);
            //OUTPUT_MSG(DEBUG, "Original Ref Vel : "<<original_ref_vel<<", sensor_car_s_speed : "<<sensor_car_s_speed);
            return min(original_ref_vel, sensor_car_s_speed * 0.9 );
          }
        }
      }
      return original_ref_vel;
    }

};

/*======================================================================================================================
                                              CLASS FUNCTION DEFINITIONS
======================================================================================================================*/
/*--------------------------------------------------------------------------------------------------------------------*/
void
PathPlanner::BuildSpline
(
 vector<double> prev_x,
 vector<double> prev_y,
 LanesEnum curr_lane,
 LanesEnum targ_lane,
 tk::spline &s_out,
 double &ref_x,
 double &ref_y,
 double &ref_yaw
)
{
  array<int, 3> way_point_offsets = {30, 60, 90};

  if(curr_lane != targ_lane)
  {
    way_point_offsets[0] = 50;
    way_point_offsets[1] = 100;
    way_point_offsets[2] = 150;
  }

  /* Pick 3 way points in the distance */
  vector<double> wp0 = getXY(_car_data[S_POS] + way_point_offsets[0], LaneToD(targ_lane, _lane_width), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);
  vector<double> wp1 = getXY(_car_data[S_POS] + way_point_offsets[1], LaneToD(targ_lane, _lane_width), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);
  vector<double> wp2 = getXY(_car_data[S_POS] + way_point_offsets[2], LaneToD(targ_lane, _lane_width), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY]);

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



/*--------------------------------------------------------------------------------------------------------------------*/
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
  double point_vel = _prev_speed[prev_x.size() + 1];
  double point_acc = _prev_acc[prev_x.size() + 1];
  LanesEnum curr_lane    = DToLane(_car_data[D_POS], _lane_width);




  /* Get a reference velocity and lane command from behaviour planner */
  double ref_vel = _B->GetRefSpeed();
  LanesEnum target_lane  = _B->GetTargetLane();
  LanesEnum curr_beh_lane = _B->GetCurrentLane();
  double ref_x;
  double ref_y;
  double ref_yaw;
  tk::spline s;
  double min_dist;

  /* If target lane is not dangerous, build spline to target lane, else to current lane
   * However if lane change has been initiated, complete it by usign the curr_beh_lane
   */
  LanesEnum spline_lane = SetSplineLaneByDanger(target_lane, curr_lane, sensor_data);
  if(curr_lane != curr_beh_lane)
  {
    /* Lane change was started previously, complete it */
    spline_lane = curr_beh_lane;
  }

  /* Build the spline based on the lane set according to danger conditions */
  BuildSpline(prev_x, prev_y, curr_lane, spline_lane, s, ref_x, ref_y, ref_yaw);

#if 0
  if(spline_lane != curr_lane)
  {
    _B->SetCurrentLane(spline_lane);
  }
#else
  _B->UpdateBehaviour(point_vel, spline_lane, sensor_data, _car_data[S_POS]);
#endif

  /* Based on reference velocity pick points along the spline as inputs */
  double target_x = 30.0;
  if(curr_lane != spline_lane)
    target_x = 60;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y * target_y));

  double x_add_on = 0;
  double s_add_on = _car_data[S_POS];
  if(prev_x.size() >= 1)
  {
    vector<double> prev_s = getFrenet(prev_x[prev_x.size() - 1], prev_y[prev_y.size()-1], ref_yaw, _map_data[X_WAY], _map_data[Y_WAY]);
    s_add_on = prev_s[0];
  }

//  OUTPUT_MSG(DEBUG, "Point vel = "<<point_vel<<", Point acc = "<<point_acc);
  for(int i = 1; i <= _max_wayp - prev_x.size(); i++)
  {
    min_dist = numeric_limits<double>::max();
    /* Check sensor fusion data to see if any cars are nearby */
    /* Start reducing speed to speed of car in fromt if in caution zone */
    /* Reduce even more if in danger zone */
    ref_vel = min(GetRefVelBasedOnSensor(ref_vel, spline_lane , sensor_data, min_dist),
                  GetRefVelBasedOnSensor(ref_vel, curr_lane , sensor_data, min_dist)
                 );

    /* Apply equations of motion for max acceleration without violating jerk */
    GetNextPointVel(point_vel, point_acc, point_vel, point_acc, ref_vel, min_dist);
    OUTPUT_MSG(DEBUG, "Point vel = "<<point_vel<<", Point acc = "<<point_acc<<", Ref Vel = "<<ref_vel);
#if 1
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

#else
    double next_s = s_add_on + (point_vel * _ts) + (0.5 * point_acc * _ts * _ts) + (_max_jerk * _ts * _ts * _ts / 6);
//    next_s = LimitS(next_s);
    vector<double> xy_temp = getXY(next_s, LaneToD(spline_lane, _lane_width), _map_data[S_WAY], _map_data[X_WAY], _map_data[Y_WAY] );
    double x_point = xy_temp[0];
    double y_point = xy_temp[1];
    OUTPUT_MSG(DEBUG, "next_s = "<<next_s<<
                      ", point_vel = "<<point_vel<<
                      ", point_acc = "<<point_acc<<
                      ", s_add_on = "<<s_add_on
                      );

    s_add_on = next_s;

    next_x.push_back(x_point);
    next_y.push_back(y_point);

#endif
    _last_acc = point_acc;
    _prev_speed[prev_x.size() + i] = point_vel;
    _prev_acc[prev_x.size() + i] = point_acc;

  }




}











#endif //_PATHPLANNER_H_
