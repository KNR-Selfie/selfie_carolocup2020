#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>

#include <math.h>

#include "road_markings_visual.h"

namespace selfie_rviz
{

float eval_poly(float x, std::vector<float> coeffs)
{
	float value = 0.0;
	for (int i = 0; i < coeffs.size(); i++)
	{
		value += std::pow(x, i) * coeffs[i];
	}

	return value;
}

RoadMarkingsVisual::RoadMarkingsVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
	scene_manager_ = scene_manager;
	frame_node_ = parent_node->createChildSceneNode();
}

RoadMarkingsVisual::~RoadMarkingsVisual()
{
	scene_manager_->destroySceneNode(frame_node_);
}

void RoadMarkingsVisual::setMessage(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{
	left_line_segments_  .clear();
	center_line_segments_.clear();
	right_line_segments_ .clear();

	Ogre::Vector3 prev_left_point, prev_center_point, prev_right_point;
	float x;
	for (x = start_x_; x < end_x_; x += step_x_)
	{
		auto left_y    = eval_poly(x, msg->left_line);
		auto center_y  = eval_poly(x, msg->center_line);
		auto right_y   = eval_poly(x, msg->right_line);

		auto left_point   = Ogre::Vector3(x, left_y,   0.0);
		auto center_point = Ogre::Vector3(x, center_y, 0.0);
		auto right_point  = Ogre::Vector3(x, right_y,  0.0);

		if (x != start_x_)
		{
			auto left_segment   = std::make_shared<rviz::Line>(scene_manager_, frame_node_);
			auto center_segment = std::make_shared<rviz::Line>(scene_manager_, frame_node_);
			auto right_segment  = std::make_shared<rviz::Line>(scene_manager_, frame_node_);

			left_segment  ->setPoints(prev_left_point,   left_point);
			center_segment->setPoints(prev_center_point, center_point);
			right_segment ->setPoints(prev_right_point,  right_point);

			left_segment  ->setColor(boundaries_color_.r,
			                         boundaries_color_.g,
			                         boundaries_color_.b,
			                         alpha_);

			center_segment->setColor(centerline_color_.r,
			                         centerline_color_.g,
			                         centerline_color_.b,
			                         alpha_);

			right_segment ->setColor(boundaries_color_.r,
			                         boundaries_color_.g,
			                         boundaries_color_.b,
			                         alpha_);

			left_line_segments_  .push_back(left_segment);
			center_line_segments_.push_back(center_segment);
			right_line_segments_ .push_back(right_segment);
		}

		prev_left_point   = left_point;
		prev_center_point = center_point;
		prev_right_point  = right_point;
	}
}

// Position and orientation are passed through to the SceneNode.
void RoadMarkingsVisual::setFramePosition(const Ogre::Vector3& position)
{
	frame_node_->setPosition(position);
}

void RoadMarkingsVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
	frame_node_->setOrientation(orientation);
}

// Color is passed through to all line segments.
void RoadMarkingsVisual::setColorsAndAlpha(Ogre::ColourValue boundaries_color,
        Ogre::ColourValue centerline_color,
        float alpha)
{
	boundaries_color_ = boundaries_color;
	centerline_color_ = centerline_color;
	alpha_ = alpha;
}

void RoadMarkingsVisual::setRenderingRangeAndStep(float start_x,
        float end_x,
        float step_x)
{
	start_x_ = start_x;
	end_x_   = end_x;
	step_x_  = step_x;
}

} // end namespace selfie_rviz
