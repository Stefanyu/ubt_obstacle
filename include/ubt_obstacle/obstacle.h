#ifndef __OBSTACLE_H
#define __OBSTACLE_H

#include <vector>

using namespace std;  

#pragma pack(1)
typedef struct
{
	float x;
	float y;
} t_Point;

typedef struct
{
    float dis;
    float theta;
} t_Scan_Point;

typedef struct
{
	float x;
	float y;
	float th;
}t_Pose;

typedef struct
{
	float lx;
	float ly;
	float rx;
	float ry;
}t_Area_Point;

typedef struct 
{
	int count;
	int num;
	float totalnum;
}t_Area_Num;

#pragma pack()

typedef vector<t_Scan_Point> t_LidarPts;
typedef vector<t_Point> t_LidarPtsTf;


enum AreaStatus
{
	stop_area = 0,
	slow_area,
	warn_area,
	max_area_status
};

enum AreaOrientation
{
	front_area = 0,
	back_area,
	left_area,
	right_area,
	max_area
};


#endif




