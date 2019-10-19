#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include "road_markings_visual.h"
#include "road_markings_display.h"

namespace selfie_rviz
{

RoadMarkingsDisplay::RoadMarkingsDisplay() : visual_(nullptr)
{
	boundaries_color_property_ = new rviz::ColorProperty("Boundaries Color", QColor(255, 255, 0),
	        "Color to draw the road boundaries.",
	        this, SLOT(updateColorsAndAlpha()));

	centerline_color_property_ = new rviz::ColorProperty("Center Line Color", QColor(255, 85, 127),
	        "Color to draw the center line.",
	        this, SLOT(updateColorsAndAlpha()));

	alpha_property_            = new rviz::FloatProperty("Alpha", 1.0,
	        "0 is fully transparent, 1.0 is fully opaque.",
	        this, SLOT(updateColorsAndAlpha()));

	rendering_start_property_  = new rviz::FloatProperty("Min X", -3.0,
	        "X coordinate at which to start rendering lines.",
	        this, SLOT(updateRenderingRange()));

	rendering_end_property_    = new rviz::FloatProperty("Max X", 3.0,
	        "X coordinate at which to end rendering lines.",
	        this, SLOT(updateRenderingRange()));

	rendering_step_property_   = new rviz::FloatProperty("Rendering Step", 0.1,
	        "Step at which subsequent line points positions should be calculated.",
	        this, SLOT(updateRenderingRange()));
}

void RoadMarkingsDisplay::onInitialize()
{
	MFDClass::onInitialize();
}

RoadMarkingsDisplay::~RoadMarkingsDisplay()
{
}

// Clear the visual by deleting its object.
void RoadMarkingsDisplay::reset()
{
	MFDClass::reset();
	if (visual_) delete visual_;
}

// Set the current colors and alpha values for the visual.
void RoadMarkingsDisplay::updateColorsAndAlpha()
{
	if (!visual_) return;

	Ogre::ColourValue boundaries_color = boundaries_color_property_->getOgreColor();
	Ogre::ColourValue centerline_color = centerline_color_property_->getOgreColor();
	float alpha = alpha_property_->getFloat();

	visual_->setColorsAndAlpha(boundaries_color, centerline_color, alpha);
}

// Set the current rendering range for the visual.
void RoadMarkingsDisplay::updateRenderingRange()
{
	if (!visual_) return;

	float start_x = rendering_start_property_->getFloat();
	float end_x   = rendering_end_property_->getFloat();
	float step_x  = rendering_step_property_->getFloat();

	visual_->setRenderingRangeAndStep(start_x, end_x, step_x);
}

// This is our callback to handle an incoming message.
void RoadMarkingsDisplay::processMessage(const selfie_msgs::RoadMarkings::ConstPtr& msg)
{
	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Imu message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
	        msg->header.stamp,
	        position, orientation))
	{
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
		          msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
		return;
	}

	// Create and configure new visual if it doesn't exist yet.
	if (!visual_)
	{
		visual_ = new RoadMarkingsVisual(context_->getSceneManager(), scene_node_);

		updateColorsAndAlpha();
		updateRenderingRange();
	}

	// Now set or update the contents of the visual.
	visual_->setMessage(msg);
	visual_->setFramePosition(position);
	visual_->setFrameOrientation(orientation);
}

} // end namespace selfie_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(selfie_rviz::RoadMarkingsDisplay, rviz::Display)
