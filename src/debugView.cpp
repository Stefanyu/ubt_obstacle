#include "debugView.h"
#include "m_printf.h"
using namespace cv;

static cv::Scalar getCvColor(const debugviz_color_e color)
{
    switch (color)
    {
    case DEBUGVIEW_COLOR_WHITE:
        return (cv::Scalar(255, 255, 255));
    case DEBUGVIEW_COLOR_RED:
        return (cv::Scalar(0, 0, 255));
    case DEBUGVIEW_COLOR_GREEN:
        return (cv::Scalar(0, 255, 0));
    case DEBUGVIEW_COLOR_BLUE:
        return (cv::Scalar(255, 0, 0));
    case DEBUGVIEW_COLOR_YELLOW:
        return (cv::Scalar(0, 255, 255));
    case DEBUGVIEW_COLOR_CYON:
        return (cv::Scalar(255, 255, 0));
    case DEBUGVIEW_COLOR_FUCHSIN:
        return (cv::Scalar(255, 0, 255));
    case DEBUGVIEW_COLOR_GRAY1:
        return (cv::Scalar(30, 30, 30));
    case DEBUGVIEW_COLOR_GRAY2:
        return (cv::Scalar(127, 127, 127));

    default:
        return (cv::Scalar(255, 255, 255));
    }
}

DebugView::DebugView()
{
    _grid_size = 1; // 4;
    _line_width = 0; // 2;

    _sizeX = 40;
    _sizeY = 40;

    _matMap = cv::Mat(_sizeY * (_grid_size + _line_width), _sizeX * (_grid_size + _line_width), CV_8UC3, cv::Scalar(0));

    _mapInfo.resolution = 10;
    _mapInfo.width = 800;
    _mapInfo.height = 800;
    _mapInfo.origin_x = 400;
    _mapInfo.origin_y = 400;
}

DebugView::~DebugView()
{
    _matMap.release();
    cv::destroyAllWindows();
}

void DebugView::mapInit(const map_info_t info)
{
    _mapInfo = info;
    resetShowMap(info.width, info.height);
}

/*显示矩形*/
void DebugView::ShowRectangle(const t_Area_Point areapt,const debugviz_color_e color)
{
    cv::Point pt1, pt2;
    // resetShowMap(_mapInfo.width, _mapInfo.height);
  

	int lx = _mapInfo.origin_x - (int)(areapt.ly / _mapInfo.resolution);
	int ly = _mapInfo.origin_y - (int)(areapt.lx / _mapInfo.resolution);
	int rx = _mapInfo.origin_x - (int)(areapt.ry / _mapInfo.resolution);
	int ry = _mapInfo.origin_y - (int)(areapt.rx / _mapInfo.resolution);

	pt1.x = lx * (_grid_size + _line_width);
	pt1.y = ly * (_grid_size + _line_width);
	pt2.x = rx * (_grid_size + _line_width);
	pt2.y = ry * (_grid_size + _line_width);

    // m_printf("pt1.x=%d pt1.y=%d",pt1.x,pt1.y);
	// m_printf("pt2.x=%d pt2.y=%d",pt2.x,pt2.y);
	if((lx > 0) && (lx < _mapInfo.width) && (ly > 0) && (ly < _mapInfo.height) && (rx > 0) &&
		(rx < _mapInfo.width) && (ry > 0) && (ry < _mapInfo.height))
    {
        cv::rectangle(_matMap, pt1, pt2, getCvColor(color), 1);
        cv::imshow("DebugViz", _matMap);
    }

//	for (uint i = 0; i < scan.size(); i++)
//	{
//		if (scan[i].dis > 0.01)
//		{
//			int x = _mapInfo.origin_x + (int)(scan[i].dis * cos(scan[i].theta) / _mapInfo.resolution);
//			int y = _mapInfo.origin_y - (int)(scan[i].dis * sin(scan[i].theta) / _mapInfo.resolution);
//            if (0 < x && x < _mapInfo.width && 0 < y && y < _mapInfo.height)
//            {
//                pt1.x = x * (_grid_size + _line_width);
//                pt1.y = y * (_grid_size + _line_width);
//                pt2.x = pt1.x + _grid_size;
//                pt2.y = pt1.y + _grid_size;
//                cv::rectangle(_matMap, pt1, pt2, getCvColor(DEBUGVIEW_COLOR_RED), CV_FILLED);
//            }
//		}
//	}
//    cv::imshow("DebugViz", _matMap);
}

void DebugView::ShowLidarScan(t_LidarPtsTf lidarptstf)
{
    cv::Point pt1, pt2;
    resetShowMap(_mapInfo.width, _mapInfo.height);
    
    for(int i = 0;i < lidarptstf.size();i ++)
    {
        int x = _mapInfo.origin_x - (int)(lidarptstf[i].y / _mapInfo.resolution);
        int y = _mapInfo.origin_y - (int)(lidarptstf[i].x / _mapInfo.resolution);
        if ((0 < x) && (x < _mapInfo.width) && (0 < y) && (y < _mapInfo.height))
        {
            pt1.x = x * (_grid_size + _line_width);
            pt1.y = y * (_grid_size + _line_width);
            pt2.x = pt1.x + _grid_size;
            pt2.y = pt1.y + _grid_size;
            cv::rectangle(_matMap, pt1, pt2, getCvColor(DEBUGVIEW_COLOR_RED), CV_FILLED);
        }
    }
	
    cv::imshow("DebugViz", _matMap);
}

void DebugView::showRobotPose(t_Pose pose)
{
    cv::Point2d _rcp, rcp;  // robot center point
    cv::Point2d _rhd, rhd;  // robot head: pixel
    const float robot_radius = 0.27f;
    
    //pose物理坐标转图像坐标
    _rcp.x = _mapInfo.origin_x + (int)(pose.x / _mapInfo.resolution);
    _rcp.y = _mapInfo.origin_y - (int)(pose.y / _mapInfo.resolution); //上下颠倒
    rcp.x = _rcp.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0); //圆心像素坐标
    rcp.y = _rcp.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    //画机器人外径
    cv::circle(_matMap, rcp, (int)((_grid_size + _line_width) * robot_radius / _mapInfo.resolution),
               getCvColor(DEBUGVIEW_COLOR_WHITE), 1);

    //画机器人朝向
    float theta = -pose.th; // robot head toward X-axis
    _rhd.x = _rcp.x + (robot_radius + 0.02) * cos(theta) / _mapInfo.resolution;
    _rhd.y = _rcp.y + (robot_radius + 0.02) * sin(theta) / _mapInfo.resolution;
    rhd.x = _rhd.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    rhd.y = _rhd.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
    cv::line(_matMap, rhd, rcp, getCvColor(DEBUGVIEW_COLOR_WHITE), 1);

    cv::imshow("DebugViz", _matMap);
}


#if 0
void DebugView::showLidarScan(pose_t pose, tLidarPts scan)
{
    cv::Point pt1, pt2;
    resetShowMap(_mapInfo.width, _mapInfo.height);
    
	for (uint i = 0; i < scan.size(); i++)
	{
		if (scan[i].dis > 0.01)
		{
			int x = _mapInfo.origin_x + (int)(scan[i].dis * cos(scan[i].theta) / _mapInfo.resolution);
			int y = _mapInfo.origin_y - (int)(scan[i].dis * sin(scan[i].theta) / _mapInfo.resolution);
            if (0 < x && x < _mapInfo.width && 0 < y && y < _mapInfo.height)
            {
                pt1.x = x * (_grid_size + _line_width);
                pt1.y = y * (_grid_size + _line_width);
                pt2.x = pt1.x + _grid_size;
                pt2.y = pt1.y + _grid_size;
                cv::rectangle(_matMap, pt1, pt2, getCvColor(DEBUGVIEW_COLOR_RED), CV_FILLED);
            }
		}
	}
    cv::imshow("DebugViz", _matMap);
}

void DebugView::showRobotPose(pose_t pose)
{
    cv::Point2d _rcp, rcp;  // robot center point
    cv::Point2d _rhd, rhd;  // robot head: pixel
    const float robot_radius = 0.27f;
    
    //pose物理坐标转图像坐标
    _rcp.x = _mapInfo.origin_x + (int)(pose.x / _mapInfo.resolution);
    _rcp.y = _mapInfo.origin_y - (int)(pose.y / _mapInfo.resolution); //上下颠倒
    rcp.x = _rcp.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0); //圆心像素坐标
    rcp.y = _rcp.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    //画机器人外径
    cv::circle(_matMap, rcp, (int)((_grid_size + _line_width) * robot_radius / _mapInfo.resolution),
               getCvColor(DEBUGVIEW_COLOR_WHITE), 1);

    //画机器人朝向
    float theta = -pose.th; // robot head toward X-axis
    _rhd.x = _rcp.x + (robot_radius + 0.02) * cos(theta) / _mapInfo.resolution;
    _rhd.y = _rcp.y + (robot_radius + 0.02) * sin(theta) / _mapInfo.resolution;
    rhd.x = _rhd.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    rhd.y = _rhd.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
    cv::line(_matMap, rhd, rcp, getCvColor(DEBUGVIEW_COLOR_WHITE), 1);

    cv::imshow("DebugViz", _matMap);
}

void DebugView::showArc(pose_t centerPose, pose_t chargerPose, float r)
{
    cv::Point2d rcp, rhd;
    int colorCircle[3] = {0, 55, 0};
    //const float arcRadius = 0.27f;
    
    //pose物理坐标转图像坐标
    int x = _mapInfo.origin_x + (int)(centerPose.x / _mapInfo.resolution);
    int y = _mapInfo.origin_y - (int)(centerPose.y / _mapInfo.resolution); //上下颠倒
    rcp.x = x * (_grid_size + _line_width) + ((double)_grid_size / 2.0); //圆心坐标
    rcp.y = y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    
    //画圆
    cv::circle(_matMap, rcp, (int)((_grid_size + _line_width) * r / _mapInfo.resolution),
               getCvColor(DEBUGVIEW_COLOR_GREEN), 1);

    //画弧形朝向
    //float theta = centerPose.th; // robot head toward X-axis
    //x = _mapInfo.origin_x + (int)((centerPose.x - r * cos(theta)) / _mapInfo.resolution);
    //y = _mapInfo.origin_y - (int)((centerPose.y - r * sin(theta)) / _mapInfo.resolution);
    
    x = _mapInfo.origin_x + (int)(chargerPose.x / _mapInfo.resolution);
    y = _mapInfo.origin_y - (int)(chargerPose.y / _mapInfo.resolution);    
    
    rhd.x = x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
    rhd.y = y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
    cv::line(_matMap, rhd, rcp, getCvColor(DEBUGVIEW_COLOR_GREEN), 1);

    cv::imshow("DebugViz", _matMap);
}
#endif

void DebugView::resetShowMap()
{
    /*_grid_size = 1;构造函数
    _line_width = 0;
    _sizeX = 40;
    _sizeY = 40;
    */
    _matMap.create(_sizeY * (_grid_size + _line_width), _sizeX * (_grid_size + _line_width), CV_8UC3);
    _matMap.setTo(cv::Scalar(0, 0, 0));
}

void DebugView::show()
{
    cv::imshow("DebugView", _matMap);
}

void DebugView::waitKey(int t)
{
    cv::waitKey(t);//sleep in ms
}
