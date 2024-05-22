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

	Point::Point(SkPoint in_point){
		normalized_point = in_point;
    }
	
	void Point::container_resized(const SkMatrix& new_transformation) {
		transformed_point = new_transformation.mapPoint(normalized_point);
    }

	double Point::distance(const SkMatrix& new_transformation,SkPoint point) const {
		auto transformed_point = new_transformation.mapPoint(point);
		return (normalized_point - transformed_point).distanceToOrigin();
    }

    SkPoint Point::get_transformed_point(const SkMatrix& new_transformation) {
        if(!transformed_point)
            container_resized(new_transformation);
        return *transformed_point;
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

    sk_sp<SkFontMgr> fontMgr() {
        static bool init = false;
        static sk_sp<SkFontMgr> fontMgr = nullptr;
        if (!init) {
#if defined(SK_FONTMGR_FONTCONFIG_AVAILABLE)
            fontMgr = SkFontMgr_New_FontConfig(nullptr);
#elif defined(SK_FONTMGR_CORETEXT_AVAILABLE)
            fontMgr = SkFontMgr_New_CoreText(nullptr);
#elif defined(SK_FONTMGR_DIRECTWRITE_AVAILABLE)
            fontMgr = SkFontMgr_New_DirectWrite();
#endif
            init = true;
        }
        return fontMgr;
    }

    sk_sp<SkTypeface> defaultTypeface(){
	    static bool init = false;
	    static sk_sp<SkTypeface> typeface;

	    if(!init){
		    std::vector<SkString> families_contained;
		    families_contained.resize(fontMgr()->countFamilies());
		    size_t i = 0;
		    for(auto& fam : families_contained){
			    fontMgr()->getFamilyName(i,&fam);
			    ++i;
		    }

		    typeface = fontMgr()->matchFamilyStyle(families_contained[0].data(),SkFontStyle::Normal());
		    init = true;
	    }

	    return typeface;
    }

    constexpr uint_least8_t dicom_compliant_conversion[256] = {0,29,32,35,37,38,40,41,42,43,44,45,46,47,47,48,49,50,51,51,52,53,54,54,55,56,57,57,58,59,60,60,61,62,62,63,63,64,65,65,66,67,67,68,69,69,70,71,71,72,73,73,74,75,75,76,77,77,78,79,79,80,80,81,82,82,83,84,84,85,86,86,87,88,89,89,90,91,91,92,93,93,94,95,95,96,97,97,98,99,99,100,101,101,102,103,104,104,105,106,107,107,108,109,110,110,111,112,112,113,114,114,115,116,117,117,118,119,120,120,121,122,123,123,124,125,126,127,127,128,129,129,130,131,132,133,133,134,135,136,137,138,138,139,140,141,142,143,143,144,145,146,147,147,148,149,150,151,152,153,154,154,155,156,157,158,159,160,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,175,176,177,178,179,180,181,182,183,184,185,186,188,188,189,191,191,192,193,194,195,196,197,199,199,201,202,202,204,205,206,207,208,209,210,212,212,214,215,216,217,218,220,220,222,223,224,225,226,227,228,230,231,232,233,235,236,237,238,240,241,242,243,244,246,247,248,250,251,253,254,255};


    sk_sp<SkColorFilter> compliantDicomTransform(){
        static auto filter = SkColorFilters::Table(dicom_compliant_conversion);
        return filter;
    }

    }
}