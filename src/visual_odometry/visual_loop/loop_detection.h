#pragma once

#include "parameters.h"
#include "keyframe.h"

class LoopDetector
{
public:

    BriefDatabase db;
    BriefVocabulary* voc;

	std::map<int, cv::Mat> image_pool;

	std::list<KeyFrame*> keyframelist;

	LoopDetector();
	void loadVocabulary(std::string voc_path);
	
	void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void addKeyFrameIntoVoc(KeyFrame* keyframe);
	KeyFrame* getKeyFrame(int index);

	void visualizeKeyPoses(double time_cur);

	int detectLoop(KeyFrame* keyframe, int frame_index);
};
