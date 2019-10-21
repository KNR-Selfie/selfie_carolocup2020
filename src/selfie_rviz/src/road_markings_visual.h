#ifndef ROAD_MARKINGS_VISUAL_H
#define ROAD_MARKINGS_VISUAL_H

#include <selfie_msgs/RoadMarkings.h>

namespace Ogre
{
class Vector3;
class Quaternion;
class ColourValue;
}

namespace rviz
{
class Line;
}

namespace selfie_rviz
{

// Each instance of RoadMarkingsVisual represents the visualization
// of a single selfie_msgs::RoadMarkings message. For now each of the three
// road markings is represented by a sequence of straight line segments.
class RoadMarkingsVisual
{
public:
	RoadMarkingsVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
	virtual ~RoadMarkingsVisual();

	// Configure the visual to show the data in the message.
	void setMessage(const selfie_msgs::RoadMarkings::ConstPtr& msg);

	// Set the pose of the coordinate frame the message refers to.
	void setFramePosition(const Ogre::Vector3& position);
	void setFrameOrientation(const Ogre::Quaternion& orientation);

	// Set the colors and alpha of the visual, which are user-editable
	// parameters and therefore don't come from the RoadMarkings message.
	void setColorsAndAlpha(Ogre::ColourValue boundaries_color,
	                       Ogre::ColourValue centerline_color,
	                       float alpha);

	// Set rendering range for the lines.
	void setRenderingRangeAndStep(float start_x, float end_x, float step_x);

private:
	float start_x_;
	float end_x_;
	float step_x_;

	Ogre::ColourValue boundaries_color_;
	Ogre::ColourValue centerline_color_;
	float alpha_;

	// Objects representing the actual lines to be displayed.
	std::vector<std::shared_ptr<rviz::Line>> left_line_segments_;
	std::vector<std::shared_ptr<rviz::Line>> center_line_segments_;
	std::vector<std::shared_ptr<rviz::Line>> right_line_segments_;

	// A SceneNode whose pose is set to match the coordinate frame of
	// the Imu message header.
	Ogre::SceneNode* frame_node_;

	// The SceneManager, kept here only so the destructor can ask it to
	// destroy the ``frame_node_``.
	Ogre::SceneManager* scene_manager_;
};

} // end namespace selfie_rviz

#endif // ROAD_MARKINGS_VISUAL_H
