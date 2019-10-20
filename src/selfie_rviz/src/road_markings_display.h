#ifndef ROAD_MARKINGS_DISPLAY_H
#define ROAD_MARKINGS_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>
#include <selfie_msgs/RoadMarkings.h>
#endif

namespace rviz
{
class ColorProperty;
class FloatProperty;
}

namespace selfie_rviz
{

class RoadMarkingsVisual;

class RoadMarkingsDisplay: public rviz::MessageFilterDisplay<selfie_msgs::RoadMarkings>
{
	Q_OBJECT
public:
	RoadMarkingsDisplay();
	virtual ~RoadMarkingsDisplay();

protected:
	virtual void onInitialize();
	virtual void reset();

private Q_SLOTS:
	void updateColorsAndAlpha();
	void updateRenderingRange();

private:
	void processMessage(const selfie_msgs::RoadMarkings::ConstPtr& msg);

	RoadMarkingsVisual* visual_;

	// User-editable property variables.
	rviz::ColorProperty* boundaries_color_property_;
	rviz::ColorProperty* centerline_color_property_;
	rviz::FloatProperty* alpha_property_;
	rviz::FloatProperty* rendering_start_property_;
	rviz::FloatProperty* rendering_end_property_;
	rviz::FloatProperty* rendering_step_property_;
};

} // end namespace selfie_rviz

#endif // ROAD_MARKINGS_DISPLAY_H
