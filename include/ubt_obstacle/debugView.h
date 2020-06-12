#ifndef DEBUGVIEW_H
#define DEBUGVIEW_H

#include "opencv2/opencv.hpp"
#include <cmath>
#include "obstacle.h"
using namespace cv;

typedef enum {
    DEBUGVIEW_COLOR_WHITE,
    DEBUGVIEW_COLOR_RED,
    DEBUGVIEW_COLOR_GREEN,
    DEBUGVIEW_COLOR_BLUE,
    DEBUGVIEW_COLOR_YELLOW,
    DEBUGVIEW_COLOR_CYON,
    DEBUGVIEW_COLOR_FUCHSIN,
    DEBUGVIEW_COLOR_GRAY1,
    DEBUGVIEW_COLOR_GRAY2,
} debugviz_color_e;

typedef struct
{
    float resolution;
    int width;
    int height;
    int origin_x;
    int origin_y;
} map_info_t;

class DebugView {
public:
    DebugView();
    ~DebugView();

    void mapInit(const map_info_t info);
	
	// void ShowRectangle(t_Point lpt,t_Point rpt);
    void ShowRectangle(const t_Area_Point areapt,const debugviz_color_e color);
    void showRobotPose(t_Pose pose);
    void ShowLidarScan(t_LidarPtsTf lidarptstf);
//    void showArc(pose_t centerPose, pose_t chargerPose, float r);

    void show();
    void waitKey(int t);

private:
    void resetShowMap();

    void resetShowMap(const size_t size_x, const size_t size_y) {
        _sizeX = size_x;
        _sizeY = size_y;
        resetShowMap();
    }

    size_t _grid_size; // in pixel
    size_t _line_width; // in pixel

    cv::Mat _matMap;//矩阵地图，应该是类mat表示一个n维密集数值单通道或多通道阵列。它可用于存储实值或复值向量和矩阵、灰度或彩色图像、体素体积、向量场、点云
    size_t _sizeX, _sizeY; // in num of grids

    map_info_t _mapInfo;
};


#endif // DEBUGVIEW_H
