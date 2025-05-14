#ifndef CURAN_INTERACTIVE_HEADER_FILE_
#define CURAN_INTERACTIVE_HEADER_FILE_

#include "UIdefinitions.h"
#include <variant>
#include <vector>
#include <Eigen/Dense>

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

struct Point{
	std::optional<SkPoint> transformed_point;
	SkPoint normalized_point;

	Point(SkPoint in_point,const SkMatrix& mat);

	Point(SkPoint in_point);
	
	void container_resized(const SkMatrix& new_transformation);

	double distance(const SkMatrix& new_transformation,SkPoint point) const;

	SkPoint get_transformed_point(const SkMatrix& new_transformation);
};

struct Path{
	SkPath rendered_path;
	SkPoint begin_point;
	std::vector<SkPoint> normalized_recorded_points;

	Path(std::vector<SkPoint> in_recorded_points,const SkMatrix& mat);

	void container_resized(const SkMatrix& new_transformation);

	double distance(const SkMatrix& new_transformation,SkPoint point) const ;
};

using Stroke = std::variant<Point,Path>;

enum Direction
{
	X = 0,
	Y = 1,
	Z = 2
};



struct directed_stroke
{	
    Eigen::Matrix<double,3,Eigen::Dynamic> point_in_image_coordinates;
    std::optional<std::array<double,3>> point; 
    curan::ui::Stroke stroke;
	Direction direction;

    directed_stroke(const Eigen::Matrix<double,3,Eigen::Dynamic>& in_points_in_image_coordinates, 
        curan::ui::Stroke in_stroke,
		Direction in_direction = Direction::Z) :  point_in_image_coordinates{in_points_in_image_coordinates},
                                                stroke{in_stroke},
												direction{in_direction}
    {}
};

sk_sp<SkFontMgr> fontMgr();

sk_sp<SkTypeface> defaultTypeface();

sk_sp<SkColorFilter> compliantDicomTransform(); 

}
}

#endif