/*
 * GridMap2D.h
 *
 *  Created on: Dec 2, 2010
 *      Author: Armin Hornung (HornungA@informatik.uni-freiburg.de)
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

	/// Returns distance at world coordinates <wx,wy> in m; -1 if out of bounds!
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
