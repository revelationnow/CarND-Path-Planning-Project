#ifndef _BEHAVIORPLANNER_H_
#define _BEHAVIORPLANNER_H_

/*======================================================================================================================
                                                    HEADER FILES
======================================================================================================================*/
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <limits>
#include "util.h"

/*======================================================================================================================
                                                     NAMESPACES
======================================================================================================================*/
using namespace std;



enum BehaviorState
{
  BEHSTAT_MAINTAIN_SPEED,
  BEHSTAT_PREP_CHANGE_LANE_LEFT,
  BEHSTAT_PREP_CHANGE_LANE_RIGHT,
  BEHSTAT_CHANGE_LANE_LEFT,
  BEHSTAT_CHANGE_LANE_RIGHT,
  BEHSTAT_MAX_STATES
};



/*======================================================================================================================
                                                  CLASS DECLARATION
======================================================================================================================*/
class BehaviorPlanner
{
  private:
    int _planning_window_size;
    int _current_index;
    BehaviorState _current_state;
    int _time_in_state;
    double _target_speed;
    double _ref_speed;
    double _lane_width;
    vector<double> _past_speeds;
    vector<LanesEnum> _past_lanes;
    LanesEnum _current_lane;
    LanesEnum _target_lane;
    vector<double> _costs;

  public:
    BehaviorPlanner()
    {
    }
    BehaviorPlanner(int window_size, double target_speed, LanesEnum start_lane, double lane_width) :
      _planning_window_size(window_size), _current_index(0), _current_state(BEHSTAT_MAINTAIN_SPEED),
      _target_speed(target_speed), _ref_speed(target_speed), _lane_width(lane_width)
    {
      srand(time(0));
      _past_speeds.resize(_planning_window_size);
      _past_lanes.resize(_planning_window_size);
      _costs.resize(LANE_MAX);
      _target_lane = start_lane;
      _current_lane = start_lane;
      _time_in_state = 0;

      for(int i = 0; i < _planning_window_size; i++)
      {
        _past_speeds[i] = 0;
        _past_lanes[i] = _current_lane;
      }

      for(int i = LANE_LEFT; i < LANE_MAX; i++)
      {
        _costs[i] = 0;
      }

      OUTPUT_MSG(INFO, "BehaviorPlanner initialized");
    }

    void SetCurrentLane(LanesEnum lane)
    {
      _current_lane = lane;
    }

    void SetTargetLane(LanesEnum lane)
    {
      _target_lane = lane;
    }

    void SetTargetSpeed(double speed)
    {
      _target_speed = speed;
    }

    LanesEnum GetTargetLane()
    {
      return _target_lane;
    }

    LanesEnum GetCurrentLane()
    {
      return _current_lane;
    }
    double GetRefSpeed()
    {
      return _ref_speed;
    }


    void UpdateBehaviour(double speed, LanesEnum lane, vector<vector<double> > sensor_data, double car_s)
    {
      _current_index = (_current_index + 1) % _planning_window_size;
      SetCurrentLane(lane);
      _past_speeds[_current_index]  = speed;
      _past_lanes[_current_index] = lane;
      if( _current_index == _planning_window_size - 1)
      {
        PlanBehavior(sensor_data, car_s, lane);
      }
    }

    void PlanBehavior(vector<vector<double>> sensor_data, double car_s, LanesEnum car_lane);

    void UpdateCosts(vector<vector<double>> sensor_data, double car_s, LanesEnum car_lane);

    double GetAverageSpeed()
    {
      double avg = 0;
      for(int i = 0; i < _planning_window_size; i++)
      {
        avg += _past_speeds[i];
      }
      return (avg/_planning_window_size);
    }

};

/*======================================================================================================================
                                              CLASS FUNCTION DEFINITIONS
======================================================================================================================*/
/*--------------------------------------------------------------------------------------------------------------------*/

void BehaviorPlanner::UpdateCosts(vector<vector<double>> sensor_data, double car_s, LanesEnum car_lane)
{
  /* Cost of multi lane change */
  double cost_multi_lane = 1000000;

  /* Cars in danger window */
  double cost_danger = 100000;
  double danger_window = 10;

  /* Cars in caution window */
  double cost_caution = 1000;
  double caution_window = 30;

  /* Cost of havign a car on the lane */
  double cost_car = 500;

  /* Cost of lane change */
  double cost_lane_change = 500;

  for(int i = LANE_LEFT; i < LANE_MAX; i++)
  {
    _costs[i] = cost_multi_lane;
  }

  _costs[car_lane] = 0;
  if(car_lane != LANE_LEFT)
    _costs[car_lane - 1] = cost_lane_change;

  if(LANE_RIGHT != car_lane)
    _costs[car_lane + 1] = cost_lane_change;


  for(int i = 0; i < sensor_data.size(); i++)
  {
    if(sensor_data[i][SENS_D] < 0)
      continue;

    if(sensor_data[i][SENS_D] > ((_lane_width * LANE_MAX) + _lane_width))
      continue;


    LanesEnum sensor_car_lane = DToLane(sensor_data[i][SENS_D], _lane_width);
    double s_dist = fabs(car_s - sensor_data[i][SENS_S]);
    if(s_dist < danger_window)
      _costs[sensor_car_lane] += cost_danger;
    else if(car_s < sensor_data[i][SENS_S])
    {
      if(s_dist < caution_window)
        _costs[sensor_car_lane] += cost_caution;
      else if(s_dist < 2 * caution_window)
        _costs[sensor_car_lane] += cost_car + 50000/s_dist;
    }
    else if(car_s < sensor_data[i][SENS_S] + caution_window)
      _costs[sensor_car_lane] += cost_car;

  }


}

void BehaviorPlanner::PlanBehavior(vector<vector<double>> sensor_data, double car_s, LanesEnum car_lane)
{
  double average_speed = GetAverageSpeed();
  OUTPUT_MSG(DEBUG, "Current state : "<<_current_state<<
                    ", Avg speed"<<average_speed<<
                    ", Time in state : "<<_time_in_state<<
                    ", Current Lane : "<<_current_lane<<
                    ", Target Lane : "<<_target_lane);
  UpdateCosts(sensor_data, car_s, car_lane);

  LanesEnum best_lane = LANE_MAX;
  double min_cost = numeric_limits<double>::max();
  for(int i = 0; i < LANE_MAX; i++)
  {
    if(_costs[i] < min_cost)
    {
      best_lane = (LanesEnum)i;
      min_cost = _costs[i];
    }
  }

  if(best_lane == car_lane)
  {
    _current_state = BEHSTAT_MAINTAIN_SPEED;
    _target_lane = car_lane;
  }
  else if( best_lane > car_lane)
  {
    _current_state = BEHSTAT_CHANGE_LANE_RIGHT;
    _target_lane = (LanesEnum)min((int)car_lane + 1, (int)LANE_RIGHT);
  }
  else
  {
    _current_state = BEHSTAT_CHANGE_LANE_LEFT;
    _target_lane = (LanesEnum)max((int)car_lane - 1, (int)LANE_LEFT);
  }




#if 0
  switch(_current_state)
  {
    case BEHSTAT_MAINTAIN_SPEED:
      _time_in_state++;
      if(average_speed >= 0.9 * _target_speed)
      {
        /* No need to do anything */
      }
      else if(_time_in_state > 10)
      {
        if(_current_lane == LANE_LEFT)
        {
          _current_state = BEHSTAT_PREP_CHANGE_LANE_RIGHT;
        }
        else if(_current_lane == LANE_RIGHT)
        {
          _current_state = BEHSTAT_PREP_CHANGE_LANE_LEFT;
        }
        else
        {
          if(rand() % 2)
          {
            _current_state = BEHSTAT_PREP_CHANGE_LANE_LEFT;
          }
          else
          {
            _current_state = BEHSTAT_PREP_CHANGE_LANE_RIGHT;
          }
        }
        _time_in_state = 0;

      }
      break;
    case BEHSTAT_PREP_CHANGE_LANE_LEFT:
      if(average_speed >= 0.9 * _target_speed)
      {
        /* Change back to maintain speed */
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
      }
      else if(average_speed < 0.8 * _target_speed)
      {
        _current_state = BEHSTAT_CHANGE_LANE_LEFT;
        _time_in_state = 0;
      }
      break;
    case BEHSTAT_PREP_CHANGE_LANE_RIGHT:
      if(average_speed >= 0.9 * _target_speed)
      {
        /* Change back to maintain speed */
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
      }
      else if(average_speed < 0.8 * _target_speed)
      {
        _current_state = BEHSTAT_CHANGE_LANE_RIGHT;
        _time_in_state = 0;
      }
      else
      {
        _time_in_state++;
      }
      break;
    case BEHSTAT_CHANGE_LANE_LEFT:
      if(_current_lane != LANE_LEFT)
      {
        if(_time_in_state == 0)
        {
          _target_lane = (LanesEnum)((int)_current_lane - 1);
        }
        if(_current_lane == _target_lane)
        {
          _current_state = BEHSTAT_MAINTAIN_SPEED;
          _time_in_state = 0;
          _ref_speed = _target_speed;
        }
        else
        {
          _time_in_state++;
#if 1
          if(_time_in_state > 20)
          {
            _current_state = BEHSTAT_MAINTAIN_SPEED;
            _target_lane = _current_lane;
            _time_in_state = 0;
          }
#endif
        }
      }
      else if(_current_lane == _target_lane)
      {
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
        _ref_speed = _target_speed;
      }
      else
      {
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
        _ref_speed = _target_speed;
        _target_lane = _current_lane;
      }
      break;
    case BEHSTAT_CHANGE_LANE_RIGHT:
      if(_current_lane != LANE_RIGHT)
      {
        if(_time_in_state == 0)
        {
          _target_lane = (LanesEnum)((int)_current_lane + 1);
        }

        if(_current_lane == _target_lane)
        {
          _current_state = BEHSTAT_MAINTAIN_SPEED;
          _time_in_state = 0;
          _ref_speed = _target_speed;
        }
        else
        {
          _time_in_state++;
#if 1
          if(_time_in_state > 20)
          {
            _current_state = BEHSTAT_MAINTAIN_SPEED;
            _target_lane = _current_lane;
            _time_in_state = 0;
          }
#endif
        }
      }
      else if(_current_lane == _target_lane)
      {
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
        _ref_speed = _target_speed;
      }
      else
      {
        _current_state = BEHSTAT_MAINTAIN_SPEED;
        _time_in_state = 0;
        _ref_speed = _target_speed;
        _target_lane = _current_lane;
      }
      break;


  }
#endif

}










#endif //_BEHAVIORPLANNER_H_
