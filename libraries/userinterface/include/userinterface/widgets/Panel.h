#ifndef CURAN_PANEL_HEADER_FILE_
#define CURAN_PANEL_HEADER_FILE_

#include "definitions/Interactive.h"
#include <unordered_map>
#include <optional>
#include "Drawable.h"
#include "utils/Lockable.h"
#include "SignalProcessor.h"
#include "IconResources.h"
#include "ImageWrapper.h"

namespace curan{
namespace ui{

class Panel : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<Panel>
{
private:

	size_t counter = 0;

	SkColor colbuton = {SK_ColorRED};
	SkPaint paint_square;
	SkPaint bluring_paint;
	SkPaint paint_stroke;
	SkPaint background_paint;
	SkPaint paint_points;

	std::unordered_map<size_t,curan::ui::Stroke> strokes;
	curan::ui::PointCollection current_stroke;

	SkRect background_rect;
	SkMatrix homogenenous_transformation;
	SkMatrix inverse_homogenenous_transformation;

	curan::ui::IconResources& system_icons;
	SkFont text_font;
	std::optional<curan::ui::ImageWrapper> background;

	std::array<float, 3> color_phase_offset;

	bool is_pressed = false;
	bool is_highlighting = false;
	curan::ui::ZoomIn zoom_in;

	SkSamplingOptions options;
	sk_sp<SkImageFilter> imgfilter;


	Panel(curan::ui::IconResources& other,std::optional<curan::ui::ImageWrapper> image_wrapper);

	void insert_in_map(const curan::ui::PointCollection& future_stroke);


public:

	static std::unique_ptr<Panel> make(curan::ui::IconResources& other,std::optional<curan::ui::ImageWrapper> image_wrapper);

	~Panel()
	{}

	void compile() override;

	void framebuffer_resize(const SkRect& new_page_size) override;

	curan::ui::drawablefunction draw() override;

	curan::ui::callablefunction call() override;
	
};

}
}

#endif