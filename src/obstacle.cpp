#include "obstacle.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "debugView.h"
#include "m_printf.h"
#define _IMAGE_DEBUG    1

// #define M_PI 3.141592653

#define OBSTACLE_FRONT_WARN_DISTANCE 1.0  //前方提醒距离
#define OBSTACLE_FRONT_SLOW_DISTANCE 0.7  //前方减速距离
#define OBSTACLE_FRONT_STOP_DISTANCE 0.3  //前方停止距离
#define OBSTACLE_RIGHT_STOP_DISTANCE 0.1  //右侧停止距离
#define OBSTACLE_LEFT_STOP_DISTANCE 0.1   //左侧停止距离

#define ROBOT_LENGTH  0.904  //机器人的长度
#define ROBOT_WIDTH   0.620  //机器人的宽度

#define LASER_TO_ROBOT_FRONT_DISTANCE 0.110  //雷达到机器人前沿的距离
#define LASER_TO_ROBOT_SIDE_DISTANCE 0.095   //雷达到机器人侧面的距离

// #define LASER_INSTALL_ANGLE -M_PI/4  //雷达安装角度 150kg
#define LASER_INSTALL_ANGLE M_PI/2  //雷达安装角度 300kg

#define LASER_POINTS_IN_AREA 1  //雷达点落在区域内的数量，避免数量太少有误判

#define LASER_MAX_RANGE 1.5  //用在避障的雷达最大范围


std_msgs::UInt8MultiArray g_AreaStatus;

t_Area_Point g_AreaPoint[max_area][max_area_status] = {1000,1000,1000,1000};

unsigned char g_Obstacle_Status = 0;

#if _IMAGE_DEBUG
DebugView debugView;//如果定义图像测试，则一个全局调试对象
#endif

template<typename value>

int sign(value data)
{
	if(data > 0)
		return 1;
	else if(data < 0)
		return -1;
	else
		return 0;
}

/*区域初始化，包括车体形状和避障的所有区域*/
void Area_Init(void)
{
	// t_Point probotl,probotr;
	t_Area_Point robotarea;
	robotarea.lx = -(ROBOT_LENGTH/2);
	robotarea.ly = (ROBOT_WIDTH/2);
	robotarea.rx = (ROBOT_LENGTH/2);
	robotarea.ry = -(ROBOT_WIDTH/2);
	
	/*front stop area point*/
	g_AreaPoint[front_area][stop_area].lx = (ROBOT_LENGTH/2);
	g_AreaPoint[front_area][stop_area].ly = (ROBOT_WIDTH/2)	+ OBSTACLE_LEFT_STOP_DISTANCE;
	g_AreaPoint[front_area][stop_area].rx = (ROBOT_LENGTH/2) + OBSTACLE_FRONT_STOP_DISTANCE;
	g_AreaPoint[front_area][stop_area].ry = -(ROBOT_WIDTH/2) - OBSTACLE_RIGHT_STOP_DISTANCE;

	/*front slow area point*/
	g_AreaPoint[front_area][slow_area].lx = (ROBOT_LENGTH/2) + OBSTACLE_FRONT_STOP_DISTANCE;
	g_AreaPoint[front_area][slow_area].ly = (ROBOT_WIDTH/2)	+ OBSTACLE_LEFT_STOP_DISTANCE;
	g_AreaPoint[front_area][slow_area].rx = (ROBOT_LENGTH/2) + OBSTACLE_FRONT_SLOW_DISTANCE;
	g_AreaPoint[front_area][slow_area].ry = -(ROBOT_WIDTH/2) - OBSTACLE_RIGHT_STOP_DISTANCE;

	/*front warn area point*/
	g_AreaPoint[front_area][warn_area].lx = (ROBOT_LENGTH/2) + OBSTACLE_FRONT_SLOW_DISTANCE;
	g_AreaPoint[front_area][warn_area].ly = (ROBOT_WIDTH/2)	+ OBSTACLE_LEFT_STOP_DISTANCE;
	g_AreaPoint[front_area][warn_area].rx = (ROBOT_LENGTH/2) + OBSTACLE_FRONT_WARN_DISTANCE;
	g_AreaPoint[front_area][warn_area].ry = -(ROBOT_WIDTH/2) - OBSTACLE_LEFT_STOP_DISTANCE;

	// /*right stop area point*/
	// g_AreaPoint[right_area][stop_area].lx = -(ROBOT_LENGTH/2);
	// g_AreaPoint[right_area][stop_area].ly = -(ROBOT_WIDTH/2);
	// g_AreaPoint[right_area][stop_area].rx = (ROBOT_LENGTH/2);
	// g_AreaPoint[right_area][stop_area].ry = -(ROBOT_WIDTH/2) - OBSTACLE_RIGHT_STOP_DISTANCE;

	/*left stop area point*/
	g_AreaPoint[right_area][stop_area].lx = -(((LASER_TO_ROBOT_SIDE_DISTANCE + OBSTACLE_LEFT_STOP_DISTANCE) * tan(1.221)) - ((ROBOT_LENGTH/2) - LASER_TO_ROBOT_FRONT_DISTANCE));
	g_AreaPoint[right_area][stop_area].ly = (ROBOT_WIDTH/2) + OBSTACLE_LEFT_STOP_DISTANCE; //1.221 laser angle_min
	g_AreaPoint[right_area][stop_area].rx = (ROBOT_LENGTH/2);
	g_AreaPoint[right_area][stop_area].ry = (ROBOT_WIDTH/2);

	#if _IMAGE_DEBUG
	if(!(g_Obstacle_Status & (0x01 << 0)))
	{
		debugView.ShowRectangle(g_AreaPoint[front_area][stop_area],DEBUGVIEW_COLOR_RED);
		debugView.ShowRectangle(g_AreaPoint[front_area][slow_area],DEBUGVIEW_COLOR_YELLOW);
		debugView.ShowRectangle(g_AreaPoint[front_area][warn_area],DEBUGVIEW_COLOR_BLUE);
	}
	if(!(g_Obstacle_Status & (0x01 << 3)))
	{
		debugView.ShowRectangle(g_AreaPoint[right_area][stop_area],DEBUGVIEW_COLOR_RED);
	}
	
	debugView.ShowRectangle(robotarea,DEBUGVIEW_COLOR_WHITE);

	debugView.waitKey(1);
	#endif

}

void Cal_Obstacle(const t_LidarPts lidarpts)
{
	static t_Area_Num areanumcal[max_area][max_area_status] = {0,0,0};
	int areanum[max_area][max_area_status] = {0};  //区域计数
	/*前方停止区域左侧的点*/

	t_LidarPtsTf lidarptstf;
	for(int i = 0;i < lidarpts.size();i ++)
	{
		t_Point curpoint;

		/*将激光坐标转换到机器人坐标系上*/
		curpoint.x = (sign(LASER_TO_ROBOT_FRONT_DISTANCE)*(ROBOT_LENGTH/2)) - LASER_TO_ROBOT_FRONT_DISTANCE + lidarpts[i].dis * cos(lidarpts[i].theta + LASER_INSTALL_ANGLE);
		curpoint.y = (sign(LASER_TO_ROBOT_SIDE_DISTANCE)*(ROBOT_WIDTH/2)) - LASER_TO_ROBOT_SIDE_DISTANCE + lidarpts[i].dis * sin(lidarpts[i].theta + LASER_INSTALL_ANGLE);
		
		lidarptstf.push_back(curpoint);

		for(int j = 0;j < max_area;j ++)
		{
			for(int k = 0;k < max_area_status;k ++)
			{
				if((curpoint.x >= g_AreaPoint[j][k].lx) && (curpoint.y <= g_AreaPoint[j][k].ly) && 
				(curpoint.x <= g_AreaPoint[j][k].rx) && (curpoint.y >= g_AreaPoint[j][k].ry))
				{
					// areanumcal[j][k].num ++;
					areanum[j][k] ++;
					break;
				}
			}
		}
	}

	// for(int j = 0;j < max_area;j ++)
	// {
	// 	for(int k = max_area_status - 1;k >= 0;k --)
	// 	{
	// 		areanumcal[j][k].count ++;
	// 		areanumcal[j][k].totalnum += areanumcal[j][k].num;
	// 		areanumcal[j][k].num = 0;
			
	// 		if(areanumcal[j][k].count >= 5)
	// 		{
	// 			areanum[j][k] = areanumcal[j][k].totalnum / areanumcal[j][k].count;

	// 			areanumcal[j][k].count = 0;
	// 			areanumcal[j][k].totalnum = 0;
	// 			if(areanum[j][k] >= LASER_POINTS_IN_AREA)
	// 			{
	// 				g_AreaStatus.data[j] = (unsigned char)k+1;
	// 			}
	// 			else
	// 			{
	// 				g_AreaStatus.data[j] = 0;
	// 			}

	// 			m_printf("areanum[%d][%d] = %f",j,k,areanum[j][k]);
	// 		}
	// 	}
	// }
	for(int j = 0;j < max_area;j ++)
	{
		// for(int k = max_area_status - 1;k >= 0;k --)
		for(int k = 0;k < max_area_status;k ++)
		{
			// areanumcal[j][k].count ++;
			// areanumcal[j][k].totalnum += areanumcal[j][k].num;
			// areanumcal[j][k].num = 0;
			
			// if(areanumcal[j][k].count >= 5)
			// {
			// 	areanum[j][k] = areanumcal[j][k].totalnum / areanumcal[j][k].count;

			// 	areanumcal[j][k].count = 0;
			// 	areanumcal[j][k].totalnum = 0;
			if(areanum[j][k] >= LASER_POINTS_IN_AREA)
			{
				g_AreaStatus.data[j] = (unsigned char)k+1;
				break;
			}
			else
			{
				g_AreaStatus.data[j] = 0;
			}

			m_printf("areanum[%d][%d] = %d",j,k,areanum[j][k]);
			// }
		}
	}

	#if _IMAGE_DEBUG
		debugView.ShowLidarScan(lidarptstf);
		Area_Init();
	#endif
}


void Laser_Callback(const sensor_msgs::LaserScan& scan)
{
	t_Scan_Point pt;
	t_LidarPts lidarpts;
	lidarpts.clear();
	
    int count = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;

	for(int i = 0;i < count;i ++)
	{
		if((scan.ranges[i] > scan.range_min) && (scan.ranges[i] < LASER_MAX_RANGE))
		{
			pt.dis = scan.ranges[i];
            pt.theta = scan.angle_min + scan.angle_increment * i;
            lidarpts.push_back(pt);
		}
	}
	
	Cal_Obstacle(lidarpts);
}

void Obstacle_Close_Callback(const std_msgs::UInt8& status)
{
	g_Obstacle_Status = status.data;
	m_printf("g_Obstacle_Status = %d",g_Obstacle_Status);
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "obstacle_node");

	ros::NodeHandle n;
	std_msgs::UInt8MultiArray areastatuslast;
	// bool pubflag = false;
	
	ros::Rate loop_rate(100);

	ros::Publisher pub_obstacle  = n.advertise<std_msgs::UInt8MultiArray>("nav_obstacle", 1);
	ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8>("nav_obstacle_status",1);  //test

    ros::Subscriber sub_obstacle_close = n.subscribe("nav_obstacle_status", 1,Obstacle_Close_Callback);
    ros::Subscriber sub_laser = n.subscribe("r2000_node/scan", 1,Laser_Callback);

	g_AreaStatus.data.resize(max_area);
	areastatuslast.data.resize(max_area);
	memset(&areastatuslast.data[0],0,g_AreaStatus.data.size());
	t_Pose pose = {0,0,0};
    map_info_t mapInfo = {0.01f, 800, 800, 400, 400};//resolution;width;height;origin_x;origin_y;
    #if _IMAGE_DEBUG
    debugView.mapInit(mapInfo);//测试地图初始化
    debugView.waitKey(1);
	#endif
	Area_Init();
	
	while(ros::ok())
	{
		for(int i = 0;i < max_area;i ++)
		{
			if(!(g_Obstacle_Status & (0x01 << i)))
			{
				// if(g_AreaStatus.data[i] != areastatuslast.data[i])
				// {
				// 	pubflag = true;
				areastatuslast.data[i] = g_AreaStatus.data[i];
				m_printf("areastatuslast.data[%d] = %d",i,areastatuslast.data[i]);
				// }
			}
			else
			{
				// pubflag = true;
				areastatuslast.data[i] = 0;
			}
		}
		// if(pubflag == true)
		// {	
		pub_obstacle.publish(areastatuslast);
		// 	pubflag = false;
		// }
		#if _IMAGE_DEBUG
		debugView.waitKey(1);
		#endif
        ros::spinOnce();       
		loop_rate.sleep();
	}
	return 0;
}








