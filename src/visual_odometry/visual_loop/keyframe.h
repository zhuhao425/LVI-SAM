#pragma once

#include "parameters.h"

#define MIN_LOOP_NUM 25

class KeyFrame
{
public:

	double time_stamp; 
	int index;

	// Pose in vins_world
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;

	cv::Mat image;
	cv::Mat thumbnail;

	std::vector<cv::Point3f> point_3d; 
	std::vector<cv::Point2f> point_2d_uv;
	std::vector<cv::Point2f> point_2d_norm;
	std::vector<double> point_id;

	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::KeyPoint> keypoints_norm;
	std::vector<cv::KeyPoint> window_keypoints;

	std::vector<DVision::BRIEF::bitset> brief_descriptors;
	std::vector<DVision::BRIEF::bitset> window_brief_descriptors;

	KeyFrame(double _time_stamp, int _index, 
			 Eigen::Vector3d &_vio_T_w_i, Eigen::Matrix3d &_vio_R_w_i, 
			 cv::Mat &_image,
			 std::vector<cv::Point3f> &_point_3d, 
			 std::vector<cv::Point2f> &_point_2d_uv, std::vector<cv::Point2f> &_point_2d_normal, 
			 std::vector<double> &_point_id);

	bool findConnection(KeyFrame* old_kf);
	void computeWindowBRIEFPoint();
	void computeBRIEFPoint();

	int HammingDis(const DVision::BRIEF::bitset &a, const DVision::BRIEF::bitset &b);

	bool searchInAera(const DVision::BRIEF::bitset window_descriptor,
	                  const std::vector<DVision::BRIEF::bitset> &descriptors_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old,
	                  const std::vector<cv::KeyPoint> &keypoints_old_norm,
	                  cv::Point2f &best_match,
	                  cv::Point2f &best_match_norm);

	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
						  std::vector<cv::Point2f> &matched_2d_old_norm,
                          std::vector<uchar> &status,
                          const std::vector<DVision::BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::KeyPoint> &keypoints_old,
                          const std::vector<cv::KeyPoint> &keypoints_old_norm);


	void PnPRANSAC(const std::vector<cv::Point2f> &matched_2d_old_norm,
	               const std::vector<cv::Point3f> &matched_3d,
	               std::vector<uchar> &status);
};