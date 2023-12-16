#include "userinterface/widgets/definitions/Interactive.h"

namespace curan
{
    namespace ui
    {

        ZoomIn::ZoomIn()
        {
            options = SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
            rectangle_paint.setStyle(SkPaint::kStroke_Style);
            rectangle_paint.setAntiAlias(true);
            rectangle_paint.setStrokeWidth(3);
            rectangle_paint.setColor(SK_ColorGREEN);
        }

        SkIRect ZoomIn::get_interger_region()
        {
            auto left = current_active_point.x() - default_zoom_in_window_size;
            if (left < 0)
                left = 0;
            auto top = current_active_point.y() - default_zoom_in_window_size;
            if (top < 0)
                top = 0;
            auto right = default_zoom_in_window_size * 2 + left;
            if (right > window_size.fRight)
                right = window_size.fRight;
            auto bottom = default_zoom_in_window_size * 2 + top;
            if (bottom > window_size.fBottom)
                bottom = window_size.fBottom;
            return SkIRect::MakeLTRB(left, top, right, bottom);
        }

        SkRect ZoomIn::get_float_region()
        {
            auto left = current_active_point.x() - default_zoom_in_window_size;
            if (left < 0)
                left = 0;
            auto top = current_active_point.y() - default_zoom_in_window_size;
            if (top < 0)
                top = 0;
            auto right = default_zoom_in_window_size * 2 + left;
            if (right > window_size.fRight)
                right = window_size.fRight;
            auto bottom = default_zoom_in_window_size * 2 + top;
            if (bottom > window_size.fBottom)
                bottom = window_size.fBottom;
            return SkRect::MakeLTRB(left, top, right, bottom);
        }

        void ZoomIn::draw(SkCanvas *canvas)
        {
            if (canvas)
            {
                auto region = get_interger_region();
                auto float_region = get_float_region();
                auto image = canvas->getSurface()->makeImageSnapshot(region);
                SkAutoCanvasRestore restore{canvas, true};
                canvas->scale(scale_factor, scale_factor);
                canvas->drawRect(SkRect::MakeXYWH(1.0 / scale_factor * (float_region.fLeft - default_zoom_in_window_size), 1.0 / scale_factor * (float_region.fTop - default_zoom_in_window_size), float_region.width(), float_region.height()), rectangle_paint);
                canvas->drawImage(image, 1.0 / scale_factor * (float_region.fLeft - default_zoom_in_window_size), 1.0 / scale_factor * (float_region.fTop - default_zoom_in_window_size), options);
            }
        }

	PointCollection::PointCollection(){
		normalized_recorded_points.reserve(1000);
		transformed_recorded_points.reserve(1000);
	}

	void PointCollection::container_resized(const SkMatrix& inverse_new_transformation){
		transformed_recorded_points = normalized_recorded_points;
		inverse_new_transformation.mapPoints(transformed_recorded_points.data(),transformed_recorded_points.size());
	}

	void PointCollection::add_point(const SkMatrix& new_transformation,SkPoint point){
		transformed_recorded_points.push_back(point);
		SkPoint other_new_point = new_transformation.mapPoint(point);
		normalized_recorded_points.push_back(other_new_point);
	}


	Point::Point(SkPoint in_point,const SkMatrix& mat){
		normalized_point = in_point;
		container_resized(mat);
    }
	
	void Point::container_resized(const SkMatrix& new_transformation){
		transformed_point = new_transformation.mapPoint(normalized_point);
    }

	double Point::distance(const SkMatrix& new_transformation,SkPoint point) const {
		auto transformed_point = new_transformation.mapPoint(point);
		return (normalized_point - transformed_point).distanceToOrigin();
    }

	Path::Path(std::vector<SkPoint> in_recorded_points,const SkMatrix& mat){
		if (in_recorded_points.size() == 0)
			return;
		normalized_recorded_points = in_recorded_points;
		container_resized(mat);
    }

	void Path::container_resized(const SkMatrix& new_transformation){
		if(normalized_recorded_points.empty())
			return;
		std::vector<SkPoint> transformed_points = normalized_recorded_points;
		new_transformation.mapPoints(transformed_points.data(),transformed_points.size());
		rendered_path.reset();
		rendered_path.moveTo(transformed_points.front());
		for (const auto &point : transformed_points)
			rendered_path.lineTo(point);
		begin_point = rendered_path.getPoint(0);
    }

	double Path::distance(const SkMatrix& new_transformation,SkPoint point) const {
		auto transformed_point = new_transformation.mapPoint(point);
		double minimum = std::numeric_limits<double>::max();
		for (const auto &r : normalized_recorded_points)
		{
			float local = (r - transformed_point).distanceToOrigin();
			if (minimum > local)
				minimum = local;
		}
		return minimum;
    }

    }
}