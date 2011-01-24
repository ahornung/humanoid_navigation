/*
 * GridMap2D.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
 */

#include "gridmap_2d/GridMap2D.h"

GridMap2D::GridMap2D()
 : m_frameId("/map")
{

}

GridMap2D::GridMap2D(const nav_msgs::OccupancyGridConstPtr& gridMap) {

	setMap(gridMap);

}

GridMap2D::~GridMap2D() {

}

void GridMap2D::setMap(const nav_msgs::OccupancyGridConstPtr& gridMap){
	m_mapInfo = gridMap->info;
	m_frameId = gridMap->header.frame_id;
	m_binaryMap = cv::Mat(m_mapInfo.width, m_mapInfo.height, CV_8UC1);
	m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);

	std::vector<signed char>::const_iterator mapDataIter = gridMap->data.begin();

	//TODO check / param
	unsigned char map_occ_thres = 70;

	for(unsigned int i = 0; i < m_mapInfo.height; ++i)	{
		for(unsigned int j = 0; j < m_mapInfo.width; ++j){
			if (*mapDataIter > map_occ_thres){
				// m_mapInfo.height-1-i
				m_binaryMap.at<uchar>(j,i) = 0;
			} else{
				m_binaryMap.at<uchar>(j,i) = 255;
			}
			mapDataIter++;
		}
	}
	cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	// distance map now contains distance in meters:
	m_distMap = m_distMap * m_mapInfo.resolution;


}


void GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
	wx = m_mapInfo.origin.position.x + (mx+0.5) * m_mapInfo.resolution;
	wy = m_mapInfo.origin.position.y + (my+0.5) * m_mapInfo.resolution;
}



void GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
    my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);
}

bool GridMap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
	if(wx < m_mapInfo.origin.position.x || m_mapInfo.origin.position.y)
		return false;

    mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
    my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);

    // TODO: x/y swap?
    if(mx < int(m_binaryMap.size().width) && my < int(m_binaryMap.size().height))
    	return true;

    return false;
}

float GridMap2D::distanceMapAt(double wx, double wy) const{
	unsigned mx, my;

	if (worldToMap(wx, wy, mx, my))
		return m_distMap.at<float>(my, mx);
	else
		return -1.0f;
}


uchar GridMap2D::binaryMapAt(double wx, double wy) const{
	unsigned mx, my;

	if (worldToMap(wx, wy, mx, my))
		return m_binaryMap.at<uchar>(my, mx);
	else
		return 0;
}


