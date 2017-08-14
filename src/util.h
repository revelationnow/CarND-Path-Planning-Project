#ifndef _UTIL_H_
#define _UTIL_H_

#include <iostream>

using namespace std;


#define STRING(x) #x
#define STRINGIFY(x) STRING(x)
#define __OUTPUT_MSG(level, ...) cout<< #level " : " __FILE__ ":" STRINGIFY(__LINE__) " == " <<__VA_ARGS__<<endl


#define OUTPUT_MSG(level, ...) OUTPUT_MSG_##level(__VA_ARGS__)


#define OUTPUT_MSG_INFO(...) __OUTPUT_MSG(INFO, __VA_ARGS__)

#ifdef FEATURE_DEBUG
#define OUTPUT_MSG_DEBUG(...) __OUTPUT_MSG(DEBUG, __VA_ARGS__)
#else
#define OUTPUT_MSG_DEBUG(...)
#endif

#define OUTPUT_MSG_ERROR(...) __OUTPUT_MSG(ERROR, __VA_ARGS__)

/* Enumerations for the car data vector fields */
enum CarData
{
  X_POS,
  Y_POS,
  S_POS,
  D_POS,
  YAW,
  SPEED,
  MAX_CARDATA
};

/* Enumerations for the Map data vector fields */
enum MapData
{
  X_WAY,
  Y_WAY,
  S_WAY,
  DX_WAY,
  DY_WAY,
  MAX_MAPDATA
};

/* Enumerations for the sensor data vector fields */
enum SensorData
{
  SENS_ID,
  SENS_X,
  SENS_Y,
  SENS_VX,
  SENS_VY,
  SENS_S,
  SENS_D,
  MAX_SENS
};


/* Value of PI */
constexpr double pi() { return M_PI; }

/* Convert Degrees to Radians */
double deg2rad(double x) { return x * pi() / 180; }

/* Convert Radians to Degrees */
double rad2deg(double x) { return x * 180 / pi(); }

/* Calculate distance between two points given the cartesian coordinates */
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/* Find the closest waypoint to a given point in the map */
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(unsigned int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

/* Find the next waypoint given a point and heading */
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

/* Transform from Cartesian x,y coordinates to Frenet s,d coordinates */
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

/* Transform from Frenet s,d coordinates to Cartesian x,y */
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/* Convert Miles per hour to metres per second */
double convert_MPH_to_mps(double MPH)
{
  return (MPH * 1600) / 3600;
}
#endif //_UTIL_H_
