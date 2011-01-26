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


#ifndef GRIDMAP2D_H_
#define GRIDMAP2D_H_

#include <opencv/cv.h>
#include <nav_msgs/OccupancyGrid.h>

class GridMap2D {
public:
	GridMap2D();
	GridMap2D(const nav_msgs::OccupancyGridConstPtr& gridMap);
	virtual ~GridMap2D();

	void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
	bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
	void worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const;
	/// Map cells to index coordinates (image indices)
	void mapToIndex(unsigned mx, unsigned my, unsigned& i, unsigned& j);
	void indexToMap(unsigned i, unsigned j, unsigned& mx, unsigned& my);

	/// Distance (in m) between two map coordinates (indices)
	inline double worldDist(unsigned x1, unsigned y1, unsigned x2, unsigned y2){
		return worldDist(cv::Point(x1, y1), cv::Point(x2, y2));
	}

	inline double worldDist(const cv::Point& p1, const cv::Point& p2){
		return GridMap2D::pointDist(p1, p2) * m_mapInfo.resolution;
	}

	/// Euclidean distance between two points:
	static inline double pointDist(const cv::Point& p1, const cv::Point& p2){
		return sqrt(pointDist2(p1, p2));
	}

	/// Squared distance between two points:
	static inline double pointDist2(const cv::Point& p1, const cv::Point& p2){
		return (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y);
	}

	/// Returns distance (in m) at world coordinates <wx,wy> in m; -1 if out of bounds!
	float distanceMapAt(double wx, double wy) const;

	/// Returns map value at world coordinates <wx,wy>; out of bounds will be returned as 0!
	uchar binaryMapAt(double wx, double wy) const;

	void setMap(const nav_msgs::OccupancyGridConstPtr& gridMap);
	inline const nav_msgs::MapMetaData& getInfo() const {return m_mapInfo;}
	inline float getResolution() const {return m_mapInfo.resolution; };
	inline const std::string getFrame() const {return m_frameId;}
	const cv::Mat& distanceMap() const {return m_distMap;}
	const cv::Mat& binaryMap() const {return m_binaryMap;}
	inline const CvSize size() const {return m_binaryMap.size();};


protected:
	cv::Mat m_binaryMap;
	cv::Mat m_distMap;
	nav_msgs::MapMetaData m_mapInfo;
	std::string m_frameId;

};

#endif /* GRIDMAP2D_H_ */
