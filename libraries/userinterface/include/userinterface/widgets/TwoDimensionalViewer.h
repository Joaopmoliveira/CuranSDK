#ifndef CURAN_TWO_DIMENSIONAL_VIEWER_HEADER_FILE_
#define CURAN_TWO_DIMENSIONAL_VIEWER_HEADER_FILE_

#include <functional>
#include "definitions/Interactive.h"
#include "ImageWrapper.h"
#include <unordered_map>
#include "Drawable.h"
#include "utils/Lockable.h"
#include "SignalProcessor.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "IconResources.h"
#include "ComputeImageBounds.h"
#include <algorithm>
#include <vector>
#include "utils/Overloading.h"
#include "utils/SafeQueue.h"
#include "itkRGBAPixel.h"
#include "itkRGBPixel.h"
#include <Eigen/Dense>

namespace curan{
namespace ui{


using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

class ItkMask
{
    std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
    ItkMask(){}
    ItkMask(const ItkMask &m) = delete;
    ItkMask &operator=(const ItkMask &) = delete;

    template <typename... T>
    std::pair<std::unordered_map<size_t, curan::ui::Stroke>::iterator, bool> try_emplace(T &&...u)
    {
        return recorded_strokes.try_emplace(std::forward<T>(u)...);
    }

    inline void erase(std::unordered_map<size_t, curan::ui::Stroke>::iterator it)
    {
        recorded_strokes.erase(it);
    }

    inline std::optional<curan::ui::Stroke> find(const size_t &key)
    {
        if (auto search = recorded_strokes.find(key); search != recorded_strokes.end())
            return search->second;
        else
            return std::nullopt;
    }

    void container_resized(const SkMatrix &inverse_homogenenous_transformation);

    inline void reset(){
        recorded_strokes = std::unordered_map<size_t, curan::ui::Stroke>{};
    }

    std::optional<curan::ui::Stroke> draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed);
};

  // Type trait to detect RGB pixels
template <typename T>
struct is_rgb_pixel : std::false_type {};
template <typename T>
struct is_rgb_pixel<itk::RGBAPixel<T>> : std::true_type {};
template <typename T>
struct is_rgb_pixel<itk::RGBPixel<T>> : std::true_type {};

template<typename imagetype>
class VolumetricMask;

template<typename imagetype>
class VolumetricMask
{
    using pressedhighlighted_event = std::function<void(VolumetricMask<imagetype>*, curan::ui::ConfigDraw*, const directed_stroke&)>;
    size_t counter = 0;
    ItkMask mask;
    imagetype::Pointer image;
public:
    std::list<pressedhighlighted_event> callbacks_pressedhighlighted;

    VolumetricMask(imagetype::Pointer volume)
    {
        update_volume(volume);
    }

    VolumetricMask(const VolumetricMask &m) = delete;
    VolumetricMask &operator=(const VolumetricMask &) = delete;

    inline void add_pressedhighlighted_call(pressedhighlighted_event &&call)
    {
        callbacks_pressedhighlighted.emplace_back(std::move(call));
    }

    inline bool filled(){
        return image.IsNotNull();
    }

    template <typename... T>
    bool try_emplace(T &&...u)
    {
        auto [iterator_to_inserted_object,insertion_successeful] = mask.try_emplace(counter, std::forward<T>(u)...);
        return insertion_successeful;
    }

    inline void update_volume(imagetype::Pointer in_volume)
    {
        image = in_volume;
        if(!filled())
            return;
        mask.reset();
    }

    inline imagetype::Pointer get_volume(){
        return image;
    }

    inline ItkMask & current_mask()
    {
        return mask;
    }
};

template<typename imagetype>
class TwoDimensionalViewer final : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<typename TwoDimensionalViewer<imagetype>>
{
public:

    const size_t size_of_slider_in_height = 30;
    const size_t buffer_around_panel = 8;

    using optionsselection_event = std::function<void(TwoDimensionalViewer<imagetype>*, curan::ui::ConfigDraw*, size_t selected_option)>;

    enum class SliderStates
    {
        WAITING,
        PRESSED,
        HOVER,
        SCROLL,
    };

    struct image_info
    {
        std::optional<curan::ui::ImageWrapper> image;
        double width_spacing = 1;
        double height_spacing = 1;
    };

private:
    SkRect reserved_slider_space;
    SkRect reserved_drawing_space;
    SkRect reserved_total_space;

    SkColor colbuton = {SK_ColorRED};

    SkPaint paint_square;
    SkPaint paint_stroke;
    SkPaint background_paint;
    SkPaint paint_points;

    SkPaint highlighted_panel;

    const double buffer_sideways = 30.0;

    VolumetricMask<typename imagetype> *volumetric_mask = nullptr;
    std::vector<directed_stroke> pending_strokes_to_process;
    curan::ui::PointCollection current_stroke;

    SkRect background_rect;
    SkMatrix homogenenous_transformation;
    SkMatrix inverse_homogenenous_transformation;

    curan::ui::IconResources &system_icons;
    SkFont text_font;

    image_info background;

    bool is_pressed = false;
    bool is_highlighting = false;
    bool is_options = false;
    curan::ui::ZoomIn zoom_in;

    std::pair<double,double> old_pressed_value;

    SkSamplingOptions options;
    std::optional<SkPaint> paint_compliant_filtered_image;

    SkColor hover_color = SK_ColorLTGRAY;
    SkColor waiting_color = SK_ColorLTGRAY;
    SkColor click_color = SK_ColorGRAY;
    SkColor slider_color = SK_ColorGRAY;
    SkColor highlight_color = SK_ColorGRAY;

    SliderStates current_state = SliderStates::WAITING;
    SkPaint slider_paint;

	size_t font_size = 20;
    sk_sp<SkImageFilter> imgfilter = SkImageFilters::Blur(20, 20, nullptr);
    SkPaint bluring_paint;
    
	SkColor options_hover_color = SK_ColorCYAN;
	SkColor options_waiting_color = SK_ColorWHITE;
	SkColor options_click_color = SK_ColorGRAY;
	SkColor options_text_color= SK_ColorBLACK;
	SkPaint options_paint;
	SkPaint options_paint_text;
    //TODO: Add paint and color options to constructor

    struct Option{
        bool is_hovering = false;
        bool is_clicked = false;
        std::string description;
        size_t index = 0;
        sk_sp<SkTextBlob> text;
        SkRect size;
        SkRect absolute_location = SkRect::MakeLTRB(0,0,0,0);
    };

    std::vector<Option> f_options;
    bool compiled = false;
    
    std::list<optionsselection_event> callbacks_optionsselection;

    struct CurrentLocationDicom{
        imagetype::IndexType image_coordinates;
        imagetype::PointType world_coordinates;
        imagetype::PixelType value;
        bool is_outside = false;
    };
    
    CurrentLocationDicom current_mouse_location;

    curan::ui::SignalInterpreter interpreter;

    image_info extract_slice_from_volume()
    {
        image_info info;
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        if constexpr (is_rgb_pixel<typename imagetype::PixelType>::value){
            std::printf("extracting rgba slice\n");
            typename imagetype::Pointer pointer_to_block_of_memory = volumetric_mask->get_volume();
            typename imagetype::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
            auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(typename imagetype::PixelType), pointer_to_block_of_memory);
            auto extracted_size = pointer_to_block_of_memory->GetBufferedRegion().GetSize();
            info.image = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[1],kRGBA_8888_SkColorType,kUnpremul_SkAlphaType};
            info.width_spacing = volumetric_mask->get_volume()->GetSpacing()[0];
            info.height_spacing = volumetric_mask->get_volume()->GetSpacing()[1];
            return info;
        } else {
            typename imagetype::Pointer pointer_to_block_of_memory = volumetric_mask->get_volume();
            typename imagetype::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
            auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(imagetype::PixelType), pointer_to_block_of_memory);
            auto extracted_size = pointer_to_block_of_memory->GetBufferedRegion().GetSize();
            info.image = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[1]};
            info.width_spacing = volumetric_mask->get_volume()->GetSpacing()[0];
            info.height_spacing = volumetric_mask->get_volume()->GetSpacing()[1];
            return info;
        }
    }

    void query_if_required(bool force_update)
    {
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        if (!volumetric_mask->filled())
            return;
        if (force_update)
        {
            background = extract_slice_from_volume();
            internal_framebuffer_recomputation();
        }
    }

    TwoDimensionalViewer(curan::ui::IconResources &other, VolumetricMask<typename imagetype> *volume_mask) : volumetric_mask{volume_mask}, system_icons{other}
    {
        set_current_state(SliderStates::WAITING);
        update_volume(volume_mask);
    
        paint_square.setStyle(SkPaint::kFill_Style);
        paint_square.setAntiAlias(true);
        paint_square.setStrokeWidth(4);
        paint_square.setColor(SK_ColorGREEN);
    
        background_paint.setStyle(SkPaint::kFill_Style);
        background_paint.setAntiAlias(true);
        background_paint.setStrokeWidth(4);
        background_paint.setColor(SK_ColorBLACK);
    
        paint_stroke.setStyle(SkPaint::kStroke_Style);
        paint_stroke.setAntiAlias(true);
        paint_stroke.setStrokeWidth(8);
        paint_stroke.setColor(SK_ColorGREEN);
    
        paint_points.setStyle(SkPaint::kFill_Style);
        paint_points.setAntiAlias(true);
        paint_points.setStrokeWidth(1);
        paint_points.setColor(SK_ColorLTGRAY);
    
        highlighted_panel.setStyle(SkPaint::kStroke_Style);
        highlighted_panel.setAntiAlias(true);
        highlighted_panel.setStrokeWidth(8);
        highlighted_panel.setColor(SK_ColorLTGRAY);
    
        options_paint.setStyle(SkPaint::kFill_Style);
        options_paint.setAntiAlias(true);
        options_paint.setStrokeWidth(4);
        options_paint.setColor(get_options_waiting_color());
    
        options_paint_text.setStyle(SkPaint::kStroke_Style);
        options_paint_text.setAntiAlias(true);
        options_paint_text.setStrokeWidth(2);
        options_paint_text.setColor(options_text_color);
    
        options = SkSamplingOptions();
    
        text_font = SkFont(curan::ui::defaultTypeface(), font_size, 1.0f, 0.0f);
        text_font.setEdging(SkFont::Edging::kAntiAlias);
    
        imgfilter = SkImageFilters::Blur(10, 10, nullptr);
        bluring_paint.setImageFilter(imgfilter);
    
    }

    void insert_in_map(const curan::ui::PointCollection &future_stroke)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        bool success = false;
        if (!volumetric_mask->filled())
            return;
        if (future_stroke.normalized_recorded_points.size() == 1)
            success = volumetric_mask->try_emplace(curan::ui::Point{future_stroke.normalized_recorded_points[0], inverse_homogenenous_transformation});
        else
            success = volumetric_mask->try_emplace(curan::ui::Path{future_stroke.normalized_recorded_points, inverse_homogenenous_transformation});
    }

public:
    static std::unique_ptr<TwoDimensionalViewer<imagetype>> make(curan::ui::IconResources &other, VolumetricMask<imagetype> *volume_mask)
    {
        std::unique_ptr<TwoDimensionalViewer<imagetype>> button = std::unique_ptr<TwoDimensionalViewer<imagetype>>(new TwoDimensionalViewer<imagetype>{other, volume_mask});
        return button;
    }

    ~TwoDimensionalViewer()
    {}

    void compile() override {
        if(compiled){
            throw std::runtime_error("cannot compile twice");
        }
        //TODO: 
        compiled = true;
    }

    inline TwoDimensionalViewer & add_overlay_processor(optionsselection_event event_processor){
        callbacks_optionsselection.push_back(event_processor);
        return *(this);
    }

    void update_volume(VolumetricMask<imagetype> *mask){
        std::lock_guard<std::mutex> g{get_mutex()};
        assert(mask != nullptr && "volumetric mask must be different from nullptr");
        if (!mask->filled()){
            return;
        }
            
        volumetric_mask = mask;
        query_if_required(true);
    }

    void framebuffer_resize(const SkRect &new_page_size) override{
        auto pos = get_position();
        std::lock_guard<std::mutex> g{get_mutex()};
        reserved_drawing_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fTop + buffer_around_panel, pos.fRight, pos.fBottom - size_of_slider_in_height - buffer_around_panel);
        reserved_slider_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fBottom - size_of_slider_in_height, pos.fRight, pos.fBottom - buffer_around_panel);
        reserved_total_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fTop + buffer_around_panel, pos.fRight - buffer_around_panel, pos.fBottom - buffer_around_panel);
        if (!volumetric_mask->filled())
            return;
        double width = 1;
        double height = 1;
    
        assert(background.image && "failed to assert that the optional is filled");
    
        if (background.width_spacing * (*background.image).image->width() > background.height_spacing * (*background.image).image->height())
        {
            height = (*background.image).image->height() * background.height_spacing / background.width_spacing;
            width = (*background.image).image->width();
        }
        else
        {
            height = (*background.image).image->height();
            width = (*background.image).image->width() * background.width_spacing / background.height_spacing;
        }
    
        background_rect = curan::ui::compute_bounded_rectangle(reserved_drawing_space, width, height);
        homogenenous_transformation = SkMatrix::MakeRectToRect(background_rect, SkRect::MakeWH(1.0, 1.0), SkMatrix::ScaleToFit::kFill_ScaleToFit);
        if (!homogenenous_transformation.invert(&inverse_homogenenous_transformation))
        {
            throw std::runtime_error("failure to invert matrix");
        }
    
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        volumetric_mask->current_mask().container_resized(inverse_homogenenous_transformation);
    
        set_size(pos);
        //TODO: need to compile the options location
        double max_width = 0.0;
        double max_height = 0.0;
        for(const auto& opt : f_options){
            if(opt.size.height() > max_height)
                max_height = opt.size.height();
            if(opt.size.width() > max_width)
                max_width = opt.size.width();        
        }
    
        max_width += buffer_sideways;
        max_height += buffer_sideways;
    
        double max_items_per_line = (reserved_drawing_space.width()-2.0*buffer_sideways) / max_width;
        size_t number_per_line = 1;
    
        number_per_line = std::floor(max_items_per_line);
    
        if( max_items_per_line < 1.0){
            number_per_line = 1;
        }
    
        double in_line_spacing  = (reserved_drawing_space.width()-2.0*buffer_sideways-(number_per_line-1)*max_width)/number_per_line;
        double x_location = reserved_drawing_space.fLeft+buffer_sideways;
        double y_location = reserved_drawing_space.fTop+buffer_sideways;
    
        size_t row_location_index = 0;
    
        for(auto& opt : f_options){
            opt.absolute_location = SkRect::MakeLTRB(x_location,y_location,x_location+max_width,y_location+max_height);    
            x_location += max_width+in_line_spacing;
            ++row_location_index;
            if(row_location_index > number_per_line){
                y_location += max_height;
                x_location = reserved_drawing_space.fLeft+buffer_sideways;
            }
        }  
        
        return;
    }

    void internal_framebuffer_recomputation(){
        auto pos = get_position();
        reserved_drawing_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fTop + buffer_around_panel, pos.fRight, pos.fBottom - size_of_slider_in_height - buffer_around_panel);
        reserved_slider_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fBottom - size_of_slider_in_height, pos.fRight, pos.fBottom - buffer_around_panel);
        reserved_total_space = SkRect::MakeLTRB(pos.fLeft + buffer_around_panel, pos.fTop + buffer_around_panel, pos.fRight - buffer_around_panel, pos.fBottom - buffer_around_panel);
        if (!volumetric_mask->filled())
            return;
        double width = 1;
        double height = 1;
    
        assert(background.image && "failed to assert that the optional is filled");
    
        if (background.width_spacing * (*background.image).image->width() > background.height_spacing * (*background.image).image->height())
        {
            height = (*background.image).image->height() * background.height_spacing / background.width_spacing;
            width = (*background.image).image->width();
        }
        else
        {
            height = (*background.image).image->height();
            width = (*background.image).image->width() * background.width_spacing / background.height_spacing;
        }
    
        background_rect = curan::ui::compute_bounded_rectangle(reserved_drawing_space, width, height);
        homogenenous_transformation = SkMatrix::MakeRectToRect(background_rect, SkRect::MakeWH(1.0, 1.0), SkMatrix::ScaleToFit::kFill_ScaleToFit);
        if (!homogenenous_transformation.invert(&inverse_homogenenous_transformation))
        {
            throw std::runtime_error("failure to invert matrix");
        }
    
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        volumetric_mask->current_mask().container_resized(inverse_homogenenous_transformation);
    }

    inline TwoDimensionalViewer & push_options(std::vector<std::string> options)
    {
        if(compiled){
            throw std::runtime_error("cannot compile twice");
        }
        // TODO: compile the text so that we have the option to compile code  
        size_t index = 0;
        f_options.reserve(options.size());
        for(const auto& opt : options){
            Option option;
            option.description = opt;
            option.index = index;
            text_font.measureText(opt.data(), opt.size(), SkTextEncoding::kUTF8, &option.size);
            option.text = SkTextBlob::MakeFromString(opt.c_str(), text_font);
            f_options.push_back(option);
            ++index;
        }
        return *(this);
    }

    inline SkColor get_hover_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return hover_color;
    }

    inline TwoDimensionalViewer &set_hover_color(SkColor color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        hover_color = color;
        return *(this);
    }

    inline SkColor get_waiting_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return waiting_color;
    }

    inline TwoDimensionalViewer &set_waiting_color(SkColor new_waiting_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        waiting_color = new_waiting_color;
        return *(this);
    }

    inline SkColor get_click_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return click_color;
    }

    inline TwoDimensionalViewer &set_click_color(SkColor new_click_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        click_color = new_click_color;
        return *(this);
    }

    inline SkColor get_slider_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return slider_color;
    }

    inline TwoDimensionalViewer &set_hightlight_color(SkColor new_hightlight_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        highlight_color = new_hightlight_color;
        return *(this);
    }

    inline SkColor get_hightlight_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return highlight_color;
    }

    inline TwoDimensionalViewer &set_slider_color(SkColor new_slider_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        slider_color = new_slider_color;
        return *(this);
    }

    inline SliderStates get_current_state()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return current_state;
    }

    inline TwoDimensionalViewer &set_current_state(SliderStates state)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        current_state = state;
        return *(this);
    }

    inline TwoDimensionalViewer& set_color_filter(sk_sp<SkColorFilter> filter){
        SkPaint paint;
        paint.setColorFilter(filter);
        std::lock_guard<std::mutex> g(get_mutex());
        paint_compliant_filtered_image = paint;
        return *(this);
    }

    inline TwoDimensionalViewer &set_options_hover_color(SkColor new_hightlight_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        options_hover_color = new_hightlight_color;
        return *(this);
    }

    inline SkColor get_options_hover_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return options_hover_color;
    }

    inline TwoDimensionalViewer &set_options_waiting_color(SkColor new_hightlight_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        options_waiting_color = new_hightlight_color;
        return *(this);
    }

    inline SkColor get_options_waiting_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return options_waiting_color;
    }

    inline TwoDimensionalViewer &set_options_click_color(SkColor new_hightlight_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        options_click_color = new_hightlight_color;
        return *(this);
    }

    inline SkColor get_options_click_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return options_click_color;
    }

    inline TwoDimensionalViewer &set_options_text_color(SkColor new_hightlight_color)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        options_text_color = new_hightlight_color;
        return *(this);
    }

    inline SkColor get_options_text_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return options_text_color;
    }

    curan::ui::drawablefunction draw() override{
        auto lamb = [this](SkCanvas *canvas)
        {
        if(!compiled){
            throw std::runtime_error("must compile the TwoDimensionalViewer before drawing operations");
        }
    
        if (!volumetric_mask->filled())
            return;
        auto widget_rect = get_position();
        bool is_panel_selected = false;
        {
            SkAutoCanvasRestore restore{canvas, true};
            highlighted_panel.setColor(get_hightlight_color());
    
            canvas->drawRect(reserved_total_space, background_paint);
            canvas->drawRect(reserved_total_space, highlighted_panel);
    
            if (background.image)
            {
                auto val = *background.image;
                auto image_display_surface = val.image;
                SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{1.0f / 3.0f, 1.0f / 3.0f});
                if (paint_compliant_filtered_image)
                    canvas->drawImageRect(image_display_surface, background_rect, opt, &(*paint_compliant_filtered_image));
                else
                    canvas->drawImageRect(image_display_surface, background_rect, opt);
            }
            canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.transformed_recorded_points.size(), current_stroke.transformed_recorded_points.data(), paint_stroke);
            {
                is_panel_selected = get_hightlight_color() == SkColorSetARGB(255, 125, 0, 0);
                // TODO: here I need to convert the coordinates of the last press mouse into the coordinates of the volume to render on screen
                std::lock_guard<std::mutex> g{get_mutex()};
    
                assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
                if(is_options) is_pressed = false;
                std::optional<curan::ui::Stroke> highlighted_and_pressed_stroke = volumetric_mask->current_mask().draw(canvas, inverse_homogenenous_transformation, homogenenous_transformation, zoom_in.get_coordinates(), is_highlighting && is_panel_selected, paint_stroke, paint_square, text_font, is_pressed);
                if (highlighted_and_pressed_stroke)
                {
                    is_pressed = false;
                    Eigen::Matrix<double, 3,Eigen::Dynamic> point_in_itk_coordinates;
                    std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path) {
                                                                point_in_itk_coordinates = Eigen::Matrix<double, 3,Eigen::Dynamic>::Zero(3,path.normalized_recorded_points.size());
                                                                for(size_t col = 0 ; col < path.normalized_recorded_points.size() ; ++col){
                                                                    point_in_itk_coordinates(0,col) =  (int)std::round(path.normalized_recorded_points[col].fX * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[0]-1));
                                                                    point_in_itk_coordinates(1,col) =  (int)std::round(path.normalized_recorded_points[col].fY * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[1]-1));
                                                                    point_in_itk_coordinates(2,col) = 0;
                                                                }
                                                            },
                                                            [&](const curan::ui::Point &point)
                                                            {
                                                                point_in_itk_coordinates = Eigen::Matrix<double, 3,Eigen::Dynamic>::Zero(3,1);
                                                                point_in_itk_coordinates(0,0) = (int)std::round(point.normalized_point.fX * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[0]-1));
                                                                point_in_itk_coordinates(1,0) = (int)std::round(point.normalized_point.fY * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[1]-1));
                                                                point_in_itk_coordinates(2,0) = 0;
                                                            }},
                               *highlighted_and_pressed_stroke);
                    if (pending_strokes_to_process.size() < 10)
                        pending_strokes_to_process.emplace_back(point_in_itk_coordinates, *highlighted_and_pressed_stroke);
                }
            }
    
            if (!is_options && zoom_in && get_hightlight_color() == SkColorSetARGB(255, 125, 0, 0) && interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA))
                zoom_in.draw(canvas);
    
            slider_paint.setColor(slider_color);
            slider_paint.setStyle(SkPaint::kStroke_Style);
            
            size_t increment_mask = 0;
            assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
    
            if(is_panel_selected){
                std::string pixel_value;
                if(!current_mouse_location.is_outside){
                    if constexpr (is_rgb_pixel<typename imagetype::PixelType>::value){
                        pixel_value = "Pixel value: (" + std::to_string(current_mouse_location.value[0]) + "," + std::to_string(current_mouse_location.value[1]) + "," + std::to_string(current_mouse_location.value[2])+"," + std::to_string(current_mouse_location.value[3])+")";
                    } else {
                        pixel_value = "Pixel value: " + std::to_string(current_mouse_location.value);
                    }
                } else 
                    pixel_value = "Pixel value: Out of range";
                
                std::string image_coordinates = "Image coordinates: [" + std::to_string((int)current_mouse_location.image_coordinates[0]) + " , " + std::to_string((int)current_mouse_location.image_coordinates[1]) + " , " + std::to_string((int)current_mouse_location.image_coordinates[2]) + "]";
                std::string frame_coordinates = "Frame coordinates: [" + std::to_string((int)current_mouse_location.world_coordinates[0]) + " , " + std::to_string((int)current_mouse_location.world_coordinates[1]) + " , " + std::to_string((int)current_mouse_location.world_coordinates[2]) + "]";
    
                options_paint.setColor(SK_ColorWHITE);
                canvas->drawSimpleText(pixel_value.data(),pixel_value.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-3.0*font_size,text_font,options_paint);    
                canvas->drawSimpleText(image_coordinates.data(),image_coordinates.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-2.0*font_size,text_font,options_paint);   
                canvas->drawSimpleText(frame_coordinates.data(),frame_coordinates.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-font_size,text_font,options_paint);   
            }
    
            slider_paint.setStyle(SkPaint::kFill_Style);
            switch (current_state)
            {
            case SliderStates::WAITING:
                slider_paint.setColor(get_waiting_color());
                break;
            case SliderStates::HOVER:
                slider_paint.setColor(get_hover_color());
                break;
            case SliderStates::PRESSED:
                slider_paint.setColor(get_click_color());
                break;
            case SliderStates::SCROLL:
                slider_paint.setColor(get_click_color());
                break;
            }
        }
            if(is_options){
                SkAutoCanvasRestore restore{canvas, true};
                //TODO : now I need to blur the background and render the options
                auto image = canvas->getSurface()->makeImageSnapshot();
                canvas->clipRect(get_position());
                canvas->drawImage(image, 0, 0, options, &bluring_paint);
                for(auto& opt : f_options){
                    if(opt.is_clicked)
                        options_paint.setColor(get_options_click_color());
                    else if(opt.is_hovering)
                        options_paint.setColor(get_options_hover_color());
                    else 
                        options_paint.setColor(get_options_waiting_color());
                    canvas->drawRect(opt.absolute_location,options_paint);
                    canvas->drawTextBlob(opt.text,opt.absolute_location.fLeft+buffer_sideways/2.0,opt.absolute_location.fBottom-buffer_sideways/2.0,options_paint_text);
                }
            }
        };
        return lamb;
    }

    curan::ui::callablefunction call() override{
        auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
        {
            if(!compiled){
                throw std::runtime_error("must compile the TwoDimensionalViewer before call operations");
            }
    
            for (const auto &highlighted_stroke : pending_strokes_to_process)
                for (auto &pending : volumetric_mask->callbacks_pressedhighlighted)
                    pending(volumetric_mask, config, highlighted_stroke);
    
            pending_strokes_to_process.clear();
    
            auto check_inside_fixed_area = [this](double x, double y)
            {
                return reserved_slider_space.contains(x, y);
            };
            auto check_inside_allocated_area = [this](double x, double y)
            {
                return get_position().contains(x, y);
            };
    
    
            interpreter.process(check_inside_allocated_area, check_inside_fixed_area, sig);
    
            if (interpreter.check(curan::ui::InterpreterStatus::KEYBOARD_EVENT)){
                
                set_current_state(SliderStates::WAITING);
                auto arg = std::get<curan::ui::Key>(sig);
                if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS){
                    if (zoom_in)
                        zoom_in.deactivate();
                    else
                        zoom_in.activate();
                }
    
                if (arg.key == GLFW_KEY_S && arg.action == GLFW_PRESS){
                    is_highlighting = !is_highlighting;
                    if (!current_stroke.empty()){
                        insert_in_map(current_stroke);
                        current_stroke.clear();
                    }
                }
                return false;
            }
    
            is_pressed = false;
    
            if(interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | curan::ui::InterpreterStatus::MOUSE_CLICKED_RIGHT_EVENT) ){
                set_current_state(SliderStates::PRESSED);
                is_options = !is_options;
                for(auto& opt : f_options){
                    opt.is_hovering = false;
                    opt.is_clicked = false; 
                }
                return false;
            }
    
            auto [xpos, ypos] = interpreter.last_move();
            if(background.image){
                auto point = homogenenous_transformation.mapPoint(SkPoint::Make(xpos,ypos));
                current_mouse_location.image_coordinates[0] =  (int)std::round(point.fX * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[0]-1));
                current_mouse_location.image_coordinates[1] =  (int)std::round(point.fY * (volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize()[1]-1));
                current_mouse_location.image_coordinates[2] = 0;
                volumetric_mask->get_volume()->TransformIndexToPhysicalPoint(current_mouse_location.image_coordinates,current_mouse_location.world_coordinates);
                auto size = volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize();
                if(size[0]> current_mouse_location.image_coordinates[0] && 
                    size[1]> current_mouse_location.image_coordinates[1] && 
                    size[2]> current_mouse_location.image_coordinates[2] && 
                    current_mouse_location.image_coordinates[0]>= 0 &&
                    current_mouse_location.image_coordinates[1]>= 0 &&
                    current_mouse_location.image_coordinates[2]>= 0)
                {
                    current_mouse_location.value= volumetric_mask->get_volume()->GetPixel(current_mouse_location.image_coordinates);
                    current_mouse_location.is_outside = false;
                }else{
                    current_mouse_location.value= typename imagetype::PixelType{};
                    current_mouse_location.is_outside = true;
                } 

                    
            }
    
    
            if(is_options && interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT) ){
                set_current_state(SliderStates::PRESSED);
                bool interacted = false;
                size_t index = 0;
                for(auto& opt : f_options){
                    if(opt.absolute_location.contains(xpos,ypos)){
                        opt.is_hovering = false;
                        opt.is_clicked = true;
                        interacted = true;
                        for(auto& callable : callbacks_optionsselection){
                            callable(this,config,index);
                        }
                    } else {
                        opt.is_hovering = false;
                        opt.is_clicked = false;              
                    }
                    ++index;
                }
                if(!interacted)
                    is_options = !is_options;
                return true;
            }
    
            if(is_options && interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT) ){
                set_current_state(SliderStates::PRESSED);
                for(auto& opt : f_options){
                    if(opt.absolute_location.contains(xpos,ypos)){
                        opt.is_hovering = false;
                        opt.is_clicked = true; 
                    } else {
                        opt.is_hovering = false;
                        opt.is_clicked = false;          
                    }
                }
                return true;
            }
    
            if(is_options && interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                                curan::ui::InterpreterStatus::MOUSE_MOVE_EVENT)){
                set_current_state(SliderStates::PRESSED);
                for(auto& opt : f_options){
                    if(opt.absolute_location.contains(xpos,ypos)){
                        opt.is_hovering = true;
                        opt.is_clicked = false; 
                    } else {
                        opt.is_hovering = false;
                        opt.is_clicked = false;         
                    }
                }
                return true;
            }
    
            if (interpreter.check(curan::ui::InterpreterStatus::ENTERED_ALLOCATED_AREA_EVENT))
            {
                set_current_state(SliderStates::WAITING);
                set_hightlight_color(SkColorSetARGB(255, 125, 0, 0));
                return false;
            }
    
            if (interpreter.check(curan::ui::InterpreterStatus::LEFT_ALLOCATED_AREA_EVENT))
            {
                set_current_state(SliderStates::WAITING);
                if (!current_stroke.empty())
                {
                    insert_in_map(current_stroke);
                    current_stroke.clear();
                }
                set_hightlight_color(SK_ColorDKGRAY);
                return true;
            }
    
            if (interpreter.check(curan::ui::InterpreterStatus::OUTSIDE_ALLOCATED_AREA))
            {
                set_current_state(SliderStates::WAITING);
                if (!current_stroke.empty())
                {
                    insert_in_map(current_stroke);
                    current_stroke.clear();
                }
                set_hightlight_color(SK_ColorDKGRAY);
                return false;
            }
    
            zoom_in.store_position(SkPoint::Make((float)xpos, (float)ypos), get_size());
            set_hightlight_color(SkColorSetARGB(255, 125, 0, 0));
    
            if (interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT))
                is_pressed = true;
            else
                is_pressed = false;
            
            if( interpreter.check(curan::ui::InterpreterStatus::MOUSE_UNCLICK_RIGHT_EVENT) || 
                interpreter.check(curan::ui::InterpreterStatus::MOUSE_UNCLICK_LEFT_EVENT )){
                set_current_state(SliderStates::WAITING);
                if (!current_stroke.empty())
                {
                    insert_in_map(current_stroke);
                    current_stroke.clear();
                }
                current_stroke.clear();
                return false;
            }
    
            if(interpreter.check(curan::ui::InterpreterStatus::INSIDE_FIXED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT)){
                old_pressed_value = interpreter.last_move();
            }
    
            if(!is_options && interpreter.status() & ~curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED && 
                interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_MOVE_EVENT | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT) ){
                set_current_state(SliderStates::PRESSED);
                if (!is_highlighting)
                    current_stroke.add_point(homogenenous_transformation, SkPoint::Make((float)xpos, (float)ypos));
                return true;
            }
    
            if(!is_options && interpreter.status() & ~curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED && 
                interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT) ){
                set_current_state(SliderStates::PRESSED);
                if (!is_highlighting)
                    current_stroke.add_point(homogenenous_transformation, SkPoint::Make((float)xpos, (float)ypos));
                return true;
            }
    
            if (interpreter.check(curan::ui::InterpreterStatus::INSIDE_FIXED_AREA))
            {
                set_current_state(SliderStates::HOVER);
                return true;
            }
    
            if (interpreter.check(curan::ui::InterpreterStatus::OUTSIDE_FIXED_AREA | curan::ui::InterpreterStatus::MOUSE_MOVE_EVENT))
            {
                set_current_state(SliderStates::WAITING);
                return true;
            }
    
            set_current_state(SliderStates::WAITING);
    
            return false;
        };
        return lamb;
    };
};

}
}

#endif