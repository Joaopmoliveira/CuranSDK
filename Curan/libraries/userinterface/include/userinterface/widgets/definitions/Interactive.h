#ifndef CURAN_INTERACTIVE_HEADER_FILE_
#define CURAN_INTERACTIVE_HEADER_FILE_

#include "UIdefinitions.h"
#include <vector>

namespace curan{
namespace ui{

constexpr int default_zoom_in_window_size = 100;

struct ZoomIn
{
	bool is_active = false;
	SkPoint current_active_point;
	SkRect window_size;
	SkSamplingOptions options;
	SkPaint rectangle_paint;
	double scale_factor = 2.0;

	ZoomIn();

	inline SkPoint get_coordinates()
	{
		return current_active_point;
	}

	inline operator bool()
	{
		return is_active;
	};

	inline void activate()
	{
		is_active = true;
	}

	inline void deactivate()
	{
		is_active = false;
	}

	inline void store_position(SkPoint new_active_point, SkRect new_window_size)
	{
		current_active_point = new_active_point;
		window_size = new_window_size;
	}

	SkIRect get_interger_region();

	SkRect get_float_region();

	void draw(SkCanvas *canvas);
};

struct PointCollection{
	std::vector<SkPoint> normalized_recorded_points;
	std::vector<SkPoint> transformed_recorded_points;

	PointCollection();

	void container_resized(const SkMatrix& inverse_new_transformation);

	void add_point(const SkMatrix& new_transformation,SkPoint point);

	inline void clear(){
		normalized_recorded_points.clear();
		transformed_recorded_points.clear();
	}

	inline bool empty(){
		return normalized_recorded_points.empty();
	}
};

struct Stroke
{
	SkPath rendered_path;
	SkPoint begin_point;
	std::vector<SkPoint> normalized_recorded_points;
	char identifier = 'n';

	Stroke(std::vector<SkPoint> in_recorded_points,const SkMatrix& mat);

	void container_resized(const SkMatrix& new_transformation);

	double distance(const SkMatrix& new_transformation,SkPoint point);
};

}
}

#endif