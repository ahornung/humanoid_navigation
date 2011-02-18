// SVN $HeadURL$
// SVN $Id$

/*
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
	// allocate map structs so that x/y in the world correspond to x/y in the image
	// (=> cv::Mat is rotated by 90 deg, because it's row-major!)
	m_binaryMap = cv::Mat(m_mapInfo.width, m_mapInfo.height, CV_8UC1);
	m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);

	std::vector<signed char>::const_iterator mapDataIter = gridMap->data.begin();

	//TODO check / param
	unsigned char map_occ_thres = 70;

	// iterate over map, store in image
	// (0,0) is lower left corner of OccupancyGrid
	for(unsigned int j = 0; j < m_mapInfo.height; ++j){
		for(unsigned int i = 0; i < m_mapInfo.width; ++i){
			if (*mapDataIter > map_occ_thres){
				// m_mapInfo.height-1-i
				m_binaryMap.at<uchar>(i,j) = 0;
			} else{
				m_binaryMap.at<uchar>(i,j) = 255;
			}
			mapDataIter++;
		}
	}
	cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	// distance map now contains distance in meters:
	m_distMap = m_distMap * m_mapInfo.resolution;

	ROS_INFO("GridMap2D created with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);
}

void GridMap2D::setMap(const cv::Mat& binaryMap){
	m_binaryMap = binaryMap.clone();
	m_distMap = cv::Mat(m_binaryMap.size(), CV_32FC1);

	cv::distanceTransform(m_binaryMap, m_distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	// distance map now contains distance in meters:
	m_distMap = m_distMap * m_mapInfo.resolution;

	ROS_INFO("GridMap2D copied from existing cv::Mat with %d x %d cells at %f resolution.", m_mapInfo.width, m_mapInfo.height, m_mapInfo.resolution);

}

// See costmap2D for mapToWorld / worldToMap implementations:

void GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
	wx = m_mapInfo.origin.position.x + (mx+0.5) * m_mapInfo.resolution;
	wy = m_mapInfo.origin.position.y + (my+0.5) * m_mapInfo.resolution;
}



void GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
    my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);
}

bool GridMap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
	if(wx < m_mapInfo.origin.position.x || wy < m_mapInfo.origin.position.y)
		return false;

    mx = (int) ((wx - m_mapInfo.origin.position.x) / m_mapInfo.resolution);
    my = (int) ((wy - m_mapInfo.origin.position.y) / m_mapInfo.resolution);

    if(mx < m_mapInfo.width && my < m_mapInfo.height)
    	return true;

    return false;
}

float GridMap2D::distanceMapAt(double wx, double wy) const{
	unsigned mx, my;

	if (worldToMap(wx, wy, mx, my))
		return m_distMap.at<float>(mx, my);
	else
		return -1.0f;
}


uchar GridMap2D::binaryMapAt(double wx, double wy) const{
	unsigned mx, my;

	if (worldToMap(wx, wy, mx, my))
		return m_binaryMap.at<uchar>(mx, my);
	else
		return 0;
}


float GridMap2D::distanceMapAtCell(unsigned int mx, unsigned int my) const{
	return m_distMap.at<float>(mx, my);
}


uchar GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) const{
	return m_binaryMap.at<uchar>(mx, my);
}


bool GridMap2D::isOccupiedAtCell(unsigned int mx, unsigned int my) const{
	return (m_binaryMap.at<uchar>(mx, my) < 255);
}


bool GridMap2D::isOccupiedAt(double wx, double wy) const{
	unsigned mx, my;
	if (worldToMap(wx, wy, mx, my))
		return isOccupiedAtCell(mx, my);
	else
		return true;
}


