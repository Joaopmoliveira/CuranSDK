#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"
#include <functional>
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/ImageWrapper.h"
#include <unordered_map>
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "userinterface/widgets/IconResources.h"
#include <algorithm>
#include <vector>
#include "utils/Overloading.h"
#include "utils/SafeQueue.h"
#include "geometry/Polyheadra.h"
#include "geometry/Intersection.h"
#include "userinterface/widgets/definitions/UIdefinitions.h"


using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;
using clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

enum MaskUsed
{
    CLEAN = 0,
    DIRTY
};

enum Direction
{
    X = 0,
    Y = 1,
    Z = 2
};

class Mask
{
    MaskUsed _mask_flag;
    std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
    Mask() : _mask_flag{MaskUsed::CLEAN} {}
    Mask(const Mask &m) = delete;
    Mask &operator=(const Mask &) = delete;

    template <typename... T>
    std::pair<std::unordered_map<size_t, curan::ui::Stroke>::iterator, bool> try_emplace(T &&...u)
    {
        _mask_flag = MaskUsed::DIRTY;
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

    inline operator bool() const
    {
        return _mask_flag;
    }

    std::optional<curan::ui::Stroke> draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed);
};

constexpr unsigned int Dimension = 3;

struct directed_stroke
{	
    Eigen::Matrix<double,3,Eigen::Dynamic> point_in_image_coordinates;
    std::optional<std::array<double,3>> point; 
    curan::ui::Stroke stroke;
    Direction direction;

    directed_stroke(const Eigen::Matrix<double,3,Eigen::Dynamic>& in_points_in_image_coordinates, 
        curan::ui::Stroke in_stroke , 
                    Direction in_direction) :  point_in_image_coordinates{in_points_in_image_coordinates},
                                                stroke{in_stroke},
                                                direction{in_direction}
    {}
};

class VolumetricMask;
using pressedhighlighted_event = std::function<void(VolumetricMask*, curan::ui::ConfigDraw*, const directed_stroke&)>;
using optionsselection_event = std::function<void(VolumetricMask*, curan::ui::ConfigDraw*, size_t selected_option)>;

class VolumetricMask
{

    static size_t counter;

    using PixelType = unsigned char;
    using ImageType = itk::Image<PixelType, Dimension>;

    std::vector<Mask> masks_x;
    std::vector<Mask> masks_y;
    std::vector<Mask> masks_z;

    std::vector<curan::geometry::PolyHeadra> three_dimensional_entities;

    ImageType::Pointer image;
public:
    std::list<pressedhighlighted_event> callbacks_pressedhighlighted;

    VolumetricMask(ImageType::Pointer volume);

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
    bool try_emplace(const Direction &direction, const float &along_dimension, T &&...u)
    {
        assert(along_dimension >= 0 && along_dimension <= 1 && "the received size is not between 0 and 1");
        switch (direction)
        {
        case Direction::X:
        {
            auto _current_index_x = std::round(along_dimension * (masks_x.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_x[_current_index_x].try_emplace(counter, std::forward<T>(u)...);
            /*
            We have inserted the object inside the set of masks, thus we need to query if the insertion on the other masks is also, sucessefull
            if true then we can 
            */
            bool erase = true;
            if (insertion_successeful)
            {
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                        {
                                                            erase = false;
                                                        },						  //        x                    y                          z
                                                        [&](const curan::ui::Point &point) { // (along_dimension ) point.normalized_point.fX point.normalized_point.fY
                                                            if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
                                                            {
                                                                erase = false;
                                                                auto _current_index_y = std::round(point.normalized_point.fX * (masks_y.size() - 1));
                                                                masks_y[_current_index_y].try_emplace(counter, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
                                                                auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
                                                                masks_z[_current_index_z].try_emplace(counter, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fX)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
                ++counter;
            }
            if (erase)
                masks_x[_current_index_x].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        case Direction::Y:
        {
            auto _current_index_y = std::round(along_dimension * (masks_y.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_y[_current_index_y].try_emplace(counter, std::forward<T>(u)...);
            bool erase = true;
            if (insertion_successeful)
            {
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                        {
                                                            erase = false;
                                                        },						  //        x                          y                          z
                                                        [&](const curan::ui::Point &point) { //  point.normalized_point.fX (along_dimension )   point.normalized_point.fY
                                                            if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
                                                            {
                                                                erase = false;
                                                                auto _current_index_x = std::round(point.normalized_point.fX * (masks_x.size() - 1));
                                                                masks_x[_current_index_x].try_emplace(counter, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
                                                                auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
                                                                masks_z[_current_index_z].try_emplace(counter, curan::ui::Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
                ++counter;
            }
            if (erase)
                masks_y[_current_index_y].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        case Direction::Z:
        {
            auto _current_index_z = std::round(along_dimension * (masks_z.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_z[_current_index_z].try_emplace(counter, std::forward<T>(u)...);
            bool erase = true;
            if (insertion_successeful)
            {
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                        {
                                                            erase = false;
                                                        },						  //        x                            y                          z
                                                        [&](const curan::ui::Point &point) { // point.normalized_point.fX point.normalized_point.fY (along_dimension )
                                                            if (point.normalized_point.fX >= 0 && point.normalized_point.fX <= 1 && point.normalized_point.fY >= 0 && point.normalized_point.fY <= 1)
                                                            {
                                                                erase = false;
                                                                auto _current_index_x = std::round(point.normalized_point.fX * (masks_x.size() - 1));
                                                                masks_x[_current_index_x].try_emplace(counter, curan::ui::Point{SkPoint::Make(point.normalized_point.fY, along_dimension)});
                                                                auto _current_index_y = std::round(point.normalized_point.fY * (masks_y.size() - 1));
                                                                masks_y[_current_index_y].try_emplace(counter, curan::ui::Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
                ++counter;
            }
            if (erase)
                masks_z[_current_index_z].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        default:
            throw std::runtime_error("incorrect mask direction selected");
        };
    }

    inline void update_volume(ImageType::Pointer in_volume)
    {
        image = in_volume;
        if(!filled())
            return;
        three_dimensional_entities = std::vector<curan::geometry::PolyHeadra>{};
        ImageType::RegionType inputRegion = image->GetBufferedRegion();
        masks_x = std::vector<Mask>(inputRegion.GetSize()[Direction::X]);
        masks_y = std::vector<Mask>(inputRegion.GetSize()[Direction::Y]);
        masks_z = std::vector<Mask>(inputRegion.GetSize()[Direction::Z]);
    }

    inline ImageType::Pointer get_volume(){
        return image;
    }

    inline size_t dimension(const Direction &direction) const
    {
        switch (direction)
        {
        case Direction::X:
            return masks_x.size();
        case Direction::Y:
            return masks_y.size();
        case Direction::Z:
            return masks_z.size();
        default:
            throw std::runtime_error("accessing direction with no meaning");
        };
    }

    template<typename T>
    void add_geometry(T&& geometry_to_add){
        three_dimensional_entities.emplace_back(std::forward<T>(geometry_to_add));
    }

    inline const std::vector<curan::geometry::PolyHeadra>& geometries() const{
        return three_dimensional_entities;
    }

    template <typename... T>
    void for_each(const Direction &direction, T &&...u) const
    {
        switch (direction)
        {
        case Direction::X:
            std::for_each(masks_x.begin(), masks_x.end(), std::forward<T>(u)...);
            break;
        case Direction::Y:
            std::for_each(masks_y.begin(), masks_y.end(), std::forward<T>(u)...);
            break;
        case Direction::Z:
            std::for_each(masks_z.begin(), masks_z.end(), std::forward<T>(u)...);
            break;
        };
    }

    template <typename... T>
    void for_each(const Direction &direction, T &&...u)
    {
        switch (direction)
        {
        case Direction::X:
            std::for_each(masks_x.begin(), masks_x.end(), std::forward<T>(u)...);
            break;
        case Direction::Y:
            std::for_each(masks_y.begin(), masks_y.end(), std::forward<T>(u)...);
            break;
        case Direction::Z:
            std::for_each(masks_z.begin(), masks_z.end(), std::forward<T>(u)...);
            break;
        };
    }

    inline Mask & current_mask(const Direction &direction, const size_t &along_dimension)
    {
        assert(along_dimension >= 0 && along_dimension <= masks_x.size() - 1 && "the received size is not between 0 and 1");
        switch (direction){
        case Direction::X:
        return masks_x[along_dimension];
        case Direction::Y:
        return masks_y[along_dimension];
        case Direction::Z:
        return masks_z[along_dimension];
        default : 
        throw std::runtime_error("incorrect mask direction selected");
        };
    }
};

constexpr size_t size_of_slider_in_height = 30;
constexpr size_t buffer_around_panel = 8;

class SlidingPanel final : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<SlidingPanel>
{
public:
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
    using PixelType = unsigned char;

    using ImageType = itk::Image<PixelType, Dimension>;
    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;

    ExtractFilterType::Pointer extract_filter;

    SkRect reserved_slider_space;
    SkRect reserved_drawing_space;
    SkRect reserved_total_space;

    SkColor colbuton = {SK_ColorRED};

    SkPaint paint_square;
    SkPaint paint_stroke;
    SkPaint background_paint;
    SkPaint paint_points;

    SkPaint highlighted_panel;

    const double buffer_sideways = 10.0;

    VolumetricMask *volumetric_mask = nullptr;
    std::vector<std::tuple<std::vector<SkPoint>,SkPath>> cached_polyheader_intersections;
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

    size_t _current_index = 0;
    float current_value = 0.5;
    float value_pressed = 0.5;
    float dragable_percent_size = 0.01f;
    std::pair<double,double> old_pressed_value;

    SkSamplingOptions options;
    std::optional<SkPaint> paint_compliant_filtered_image;

    SkColor hover_color = SK_ColorLTGRAY;
    SkColor waiting_color = SK_ColorLTGRAY;
    SkColor click_color = SK_ColorGRAY;
    SkColor slider_color = SK_ColorGRAY;
    SkColor highlight_color = SK_ColorGRAY;

    SliderStates current_state = SliderStates::WAITING;
    Direction direction = Direction::X;
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

    curan::ui::SignalInterpreter interpreter;

    void query_if_required(bool force_update);

    image_info extract_slice_from_volume(size_t index);

    SlidingPanel(curan::ui::IconResources &other, VolumetricMask *mask, Direction in_direction);

    void insert_in_map(const curan::ui::PointCollection &future_stroke);

public:
    static std::unique_ptr<SlidingPanel> make(curan::ui::IconResources &other, VolumetricMask *mask, Direction in_direction);

    ~SlidingPanel();

    void compile() override;

    void update_volume(VolumetricMask *mask, Direction in_direction);

    void framebuffer_resize(const SkRect &new_page_size) override;

    inline SlidingPanel &trigger(float in_current_value)
    {
        value_pressed = in_current_value;
        return *(this);
    }

    inline SlidingPanel & push_options(std::vector<std::string> options)
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

    inline float read_trigger()
    {
        return value_pressed;
    }

    inline SlidingPanel &set_current_value(float in_current_value)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        if (in_current_value < 0.0)
            in_current_value = 0.0;
        if (in_current_value > 1.0)
            in_current_value = 1.0;
        current_value = in_current_value;
        query_if_required(false);
        return *(this);
    }

    inline float get_current_value()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return current_value;
    }

    inline SkColor get_hover_color()
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        return hover_color;
    }

    inline SlidingPanel &set_hover_color(SkColor color)
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

    inline SlidingPanel &set_waiting_color(SkColor new_waiting_color)
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

    inline SlidingPanel &set_click_color(SkColor new_click_color)
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

    inline SlidingPanel &set_hightlight_color(SkColor new_hightlight_color)
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

    inline SlidingPanel &set_slider_color(SkColor new_slider_color)
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

    inline SlidingPanel &set_current_state(SliderStates state)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        current_state = state;
        return *(this);
    }

    inline SlidingPanel& set_color_filter(sk_sp<SkColorFilter> filter){
        SkPaint paint;
        paint.setColorFilter(filter);
        std::lock_guard<std::mutex> g(get_mutex());
        paint_compliant_filtered_image = paint;
        return *(this);
    }

    inline SlidingPanel &set_options_hover_color(SkColor new_hightlight_color)
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

    inline SlidingPanel &set_options_waiting_color(SkColor new_hightlight_color)
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

    inline SlidingPanel &set_options_click_color(SkColor new_hightlight_color)
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

    inline SlidingPanel &set_options_text_color(SkColor new_hightlight_color)
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

    curan::ui::drawablefunction draw() override;

    curan::ui::callablefunction call() override;
};

size_t VolumetricMask::counter = 0;

VolumetricMask::VolumetricMask(ImageType::Pointer volume) : image{volume}
{
    update_volume(volume);
}

void Mask::container_resized(const SkMatrix &inverse_homogenenous_transformation)
{
    for (auto &stro : recorded_strokes)
        std::visit(curan::utilities::overloaded{[&](curan::ui::Path &path)
                                                {
                                                    path.container_resized(inverse_homogenenous_transformation);
                                                },
                                                [&](curan::ui::Point &point)
                                                {
                                                    point.container_resized(inverse_homogenenous_transformation);
                                                }},
                   stro.second);
}

std::optional<curan::ui::Stroke> Mask::draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed)
{

    if (is_highlighting)
    {
        double minimum = std::numeric_limits<double>::max();
        auto minimum_index = recorded_strokes.end();
        for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
        {
            double local = 0.0;
            std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                    {
                                                        local = path.distance(homogenenous_transformation, point);
                                                        canvas->drawPath(path.rendered_path, paint_stroke);
                                                    },
                                                    [&](curan::ui::Point &in_point)
                                                    {
                                                        local = in_point.distance(homogenenous_transformation, point);
                                                        canvas->drawPoint(in_point.get_transformed_point(inverse_homogenenous_transformation), paint_stroke);
                                                    }},
                       begin->second);

            if (minimum > local)
            {
                minimum = local;
                minimum_index = begin;
            }
        }

        for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
            std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                    {
                                                        paint_square.setColor(SkColorSetARGB(155, 0, 0, 0));
                                                        canvas->drawCircle(SkPoint::Make(path.begin_point.fX + 10, path.begin_point.fY + 10), 20, paint_square);
                                                        paint_square.setColor(SK_ColorGREEN);
                                                        std::string indentifier = "s" + std::to_string(begin->first);
                                                        paint_stroke.setStrokeWidth(0.5f);
                                                        canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, path.begin_point.fX + 10, path.begin_point.fY + 10, text_font, paint_square);
                                                        paint_stroke.setStrokeWidth(8);
                                                    },
                                                    [&](curan::ui::Point &in_point)
                                                    {
                                                        auto local_point = in_point.get_transformed_point(inverse_homogenenous_transformation);
                                                        paint_square.setColor(SkColorSetARGB(60, 0, 0, 0));
                                                        canvas->drawCircle(SkPoint::Make(local_point.fX + 5, local_point.fY + 5), 10, paint_square);
                                                        paint_square.setColor(SK_ColorGREEN);
                                                        std::string indentifier = "p" + std::to_string(begin->first);
                                                        canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, local_point.fX + 10, local_point.fY + 10, text_font, paint_square);
                                                    }},
                       begin->second);

        if (minimum_index != recorded_strokes.end() && minimum < 0.02f)
        {
            if (is_pressed)
            {
                paint_stroke.setStrokeWidth(14);
                paint_stroke.setColor(SkColorSetARGB(0xFF, 0xFF, 0x00, 0x00));
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                        {
                                                            canvas->drawPath(path.rendered_path, paint_stroke);
                                                        },
                                                        [&](curan::ui::Point &in_point)
                                                        {
                                                            canvas->drawPoint(in_point.get_transformed_point(inverse_homogenenous_transformation), paint_stroke);
                                                        }},
                           minimum_index->second);
                paint_stroke.setStrokeWidth(8);
                paint_stroke.setColor(SK_ColorGREEN);
                return minimum_index->second;
            }
            else
            {
                paint_stroke.setStrokeWidth(14);
                paint_stroke.setColor(SkColorSetARGB(125, 0x00, 0xFF, 0x00));
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                        {
                                                            canvas->drawPath(path.rendered_path, paint_stroke);
                                                        },
                                                        [&](curan::ui::Point &in_point)
                                                        {
                                                            canvas->drawPoint(in_point.get_transformed_point(inverse_homogenenous_transformation), paint_stroke);
                                                        }},
                           minimum_index->second);
                paint_stroke.setStrokeWidth(8);
                paint_stroke.setColor(SK_ColorGREEN);
            }
        }
    }
    else
    {
        for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
            std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                    {
                                                        canvas->drawPath(path.rendered_path, paint_stroke);
                                                    },
                                                    [&](curan::ui::Point &in_point)
                                                    {
                                                        canvas->drawPoint(in_point.get_transformed_point(inverse_homogenenous_transformation), paint_stroke);
                                                    }},
                       begin->second);

        for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
            std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path)
                                                    {
                                                        auto point = path.begin_point;
                                                        paint_square.setColor(SkColorSetARGB(155, 0, 0, 0));
                                                        canvas->drawCircle(SkPoint::Make(point.fX + 10, point.fY + 10), 20, paint_square);
                                                        paint_square.setColor(SK_ColorGREEN);
                                                        std::string indentifier = "s" + std::to_string(begin->first);
                                                        paint_stroke.setStrokeWidth(0.5f);
                                                        canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
                                                        paint_stroke.setStrokeWidth(8);
                                                    },
                                                    [&](curan::ui::Point &in_point)
                                                    {
                                                        auto point = in_point.get_transformed_point(inverse_homogenenous_transformation);
                                                        paint_square.setColor(SkColorSetARGB(60, 0, 0, 0));
                                                        canvas->drawCircle(SkPoint::Make(point.fX + 5, point.fY + 5), 10, paint_square);
                                                        paint_square.setColor(SK_ColorGREEN);
                                                        std::string indentifier = "p" + std::to_string(begin->first);
                                                        canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
                                                    }},
                       begin->second);
    }
    return std::nullopt;
}

void SlidingPanel::query_if_required(bool force_update)
{
    size_t previous = _current_index;
    assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
    if (!volumetric_mask->filled())
        return;
    _current_index = std::round(current_value * (volumetric_mask->dimension(direction) - 1));
    if (force_update)
    {
        background = extract_slice_from_volume(_current_index);
    }
    else if (previous != _current_index)
        background = extract_slice_from_volume(_current_index);
    previous = _current_index;
}

SlidingPanel::image_info SlidingPanel::extract_slice_from_volume(size_t index)
{
    image_info info;
    assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");

    extract_filter = ExtractFilterType::New();
    extract_filter->SetDirectionCollapseToSubmatrix();
    extract_filter->SetInput(volumetric_mask->get_volume());

    ImageType::RegionType inputRegion = volumetric_mask->get_volume()->GetBufferedRegion();
    ImageType::SpacingType spacing = volumetric_mask->get_volume()->GetSpacing();
    ImageType::SizeType size = inputRegion.GetSize();

    auto copy_size = size;
    size[direction] = 1;

    ImageType::IndexType start = inputRegion.GetIndex();
    start[direction] = index;
    ImageType::RegionType desiredRegion;
    desiredRegion.SetSize(size);
    desiredRegion.SetIndex(start);
    extract_filter->SetExtractionRegion(desiredRegion);
    extract_filter->UpdateLargestPossibleRegion();

    ImageType::Pointer pointer_to_block_of_memory = extract_filter->GetOutput();
    ImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(PixelType), pointer_to_block_of_memory);

    auto extracted_size = pointer_to_block_of_memory->GetBufferedRegion().GetSize();

    switch (direction)
    {
    case Direction::X:
        info.image = curan::ui::ImageWrapper{buff, extracted_size[1], extracted_size[2]};
        info.width_spacing = spacing[1];
        info.height_spacing = spacing[2];
        break;
    case Direction::Y:
        info.image = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[2]};
        info.width_spacing = spacing[0];
        info.height_spacing = spacing[2];
        break;
    case Direction::Z:
        info.image = curan::ui::ImageWrapper{buff, extracted_size[0], extracted_size[1]};
        info.width_spacing = spacing[0];
        info.height_spacing = spacing[1];
        break;
    }

    /*
    The geometries are normalized in volume coordinates, thus we must
    convert them into world coordinates
    */
    cached_polyheader_intersections = std::vector<std::tuple<std::vector<SkPoint>, SkPath>>{};
    for (const auto &cliped_path : volumetric_mask->geometries())
    {
        std::vector<SkPoint> points_in_path;

        Eigen::Matrix<double, 3, 1> normal{0.0, 0.0, 0.0};
        Eigen::Matrix<double, 3, 1> origin{0.5, 0.5, 0.5};
        origin[direction] = current_value;
        normal[direction] = 1.0;

        auto possible_cliped_polygon = curan::geometry::clip_with_plane(cliped_path, normal, origin);
        if (!possible_cliped_polygon)
        {
            continue;
        }

        if ((*possible_cliped_polygon).cols() == 0)
        {
            continue;
        }

        if ((*possible_cliped_polygon).cols() < 2)
        {

            switch (direction)
            {
            case Direction::X:
                points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[1], (*possible_cliped_polygon).col(0)[2]));
                break;
            case Direction::Y:
                points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[0], (*possible_cliped_polygon).col(0)[2]));
                break;
            case Direction::Z:
                points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[0], (*possible_cliped_polygon).col(0)[1]));
                break;
            }

            continue;
        }

        switch (direction)
        {
        case Direction::X:
            points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[1], (*possible_cliped_polygon).col(0)[2]));
            break;
        case Direction::Y:
            points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[0], (*possible_cliped_polygon).col(0)[2]));
            break;
        case Direction::Z:
            points_in_path.push_back(SkPoint::Make((*possible_cliped_polygon).col(0)[0], (*possible_cliped_polygon).col(0)[1]));
            break;
        }

        for (const auto &cliped_polygon : (*possible_cliped_polygon).colwise())
        {
            switch (direction)
            {
            case Direction::X:
                points_in_path.push_back(SkPoint::Make(cliped_polygon[1], cliped_polygon[2]));
                break;
            case Direction::Y:
                points_in_path.push_back(SkPoint::Make(cliped_polygon[0], cliped_polygon[2]));
                break;
            case Direction::Z:
                points_in_path.push_back(SkPoint::Make(cliped_polygon[0], cliped_polygon[1]));
                break;
            }
        }

        std::vector<SkPoint> transformed_points = points_in_path;

        inverse_homogenenous_transformation.mapPoints(transformed_points.data(), transformed_points.size());
        SkPath polygon_intersection;
        polygon_intersection.moveTo(transformed_points.front());
        for (const auto &point : transformed_points)
            polygon_intersection.lineTo(point);
        polygon_intersection.close();

        cached_polyheader_intersections.push_back(std::make_tuple(points_in_path, polygon_intersection));
    }

    return info;
}

SlidingPanel::SlidingPanel(curan::ui::IconResources &other, VolumetricMask *volume_mask, Direction in_direction) : volumetric_mask{volume_mask}, system_icons{other}
{
    set_current_state(SliderStates::WAITING);
    update_volume(volume_mask, in_direction);

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

void SlidingPanel::insert_in_map(const curan::ui::PointCollection &future_stroke)
{
    std::lock_guard<std::mutex> g{get_mutex()};
    assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
    bool success = false;
    if (!volumetric_mask->filled())
        return;
    if (future_stroke.normalized_recorded_points.size() == 1)
        success = volumetric_mask->try_emplace(direction, current_value, curan::ui::Point{future_stroke.normalized_recorded_points[0], inverse_homogenenous_transformation});
    else
        success = volumetric_mask->try_emplace(direction, current_value, curan::ui::Path{future_stroke.normalized_recorded_points, inverse_homogenenous_transformation});
}

std::unique_ptr<SlidingPanel> SlidingPanel::make(curan::ui::IconResources &other, VolumetricMask *volume_mask, Direction in_direction)
{
    std::unique_ptr<SlidingPanel> button = std::unique_ptr<SlidingPanel>(new SlidingPanel{other, volume_mask, in_direction});
    return button;
}

SlidingPanel::~SlidingPanel()
{
}

void SlidingPanel::compile()
{
    if(compiled){
        throw std::runtime_error("cannot compile twice");
    }
    //TODO: 
    compiled = true;
}

void SlidingPanel::update_volume(VolumetricMask *volume_mask, Direction in_direction)
{
    std::lock_guard<std::mutex> g{get_mutex()};
    assert(volume_mask != nullptr && "volumetric mask must be different from nullptr");
    if (!volume_mask->filled())
        return;
    direction = in_direction;
    ImageType::RegionType inputRegion = volume_mask->get_volume()->GetBufferedRegion();
    ImageType::SizeType size = inputRegion.GetSize();
    _current_index = std::floor(current_value * (size[direction] - 1));
    volumetric_mask = volume_mask;
    dragable_percent_size = 1.0 / size[direction];
    query_if_required(true);
}

void SlidingPanel::framebuffer_resize(const SkRect &new_page_size)
{
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
    volumetric_mask->for_each(direction, [&](Mask &mask)
                              { mask.container_resized(inverse_homogenenous_transformation); });

    for (auto &[normalized_path, cached_path] : cached_polyheader_intersections)
    {
        std::vector<SkPoint> transformed_points = normalized_path;
        inverse_homogenenous_transformation.mapPoints(transformed_points.data(), transformed_points.size());
        cached_path.reset();
        cached_path.moveTo(transformed_points.front());
        for (const auto &point : transformed_points)
            cached_path.lineTo(point);
        cached_path.close();
    }

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

    double in_line_spacing  = (reserved_drawing_space.width()-2.0*buffer_sideways-number_per_line*max_width)/number_per_line;

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

curan::ui::drawablefunction SlidingPanel::draw()
{
    auto lamb = [this](SkCanvas *canvas)
    {
    if(!compiled){
        throw std::runtime_error("must compile the SlidingPanel before drawing operations");
    }

    if (!volumetric_mask->filled())
        return;
    auto widget_rect = get_position();
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
            bool is_panel_selected = get_hightlight_color() == SkColorSetARGB(255, 125, 0, 0);
            // TODO: here I need to convert the coordinates of the last press mouse into the coordinates of the volume to render on screen
            SkPaint paint_cached_paths;
            paint_cached_paths.setAntiAlias(true);
            paint_cached_paths.setStyle(SkPaint::kStrokeAndFill_Style);
            paint_cached_paths.setColor(SkColorSetARGB(100, 0xFF, 0x00, 0x00));
            auto paint_cached_paths_outline = paint_cached_paths;
            paint_cached_paths_outline.setColor(SkColorSetARGB(0xFF, 0xFF, 0x00, 0x00));
            paint_cached_paths_outline.setStyle(SkPaint::kStroke_Style);
            std::lock_guard<std::mutex> g{get_mutex()};

            for (const auto &[normalized_path, cached_path] : cached_polyheader_intersections)
            {
                canvas->drawPath(cached_path, paint_cached_paths);
                canvas->drawPath(cached_path, paint_cached_paths_outline);
            }

            assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
            if(is_options) is_pressed = false;
            std::optional<curan::ui::Stroke> highlighted_and_pressed_stroke = volumetric_mask->current_mask(direction, _current_index).draw(canvas, inverse_homogenenous_transformation, homogenenous_transformation, zoom_in.get_coordinates(), is_highlighting && is_panel_selected, paint_stroke, paint_square, text_font, is_pressed);
            if (highlighted_and_pressed_stroke)
            {
                is_pressed = false;
                Eigen::Matrix<double, 3,Eigen::Dynamic> point_in_itk_coordinates;
                std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path) {
                                                            point_in_itk_coordinates = Eigen::Matrix<double, 3,Eigen::Dynamic>::Zero(3,path.normalized_recorded_points.size());
                                                            switch (direction)
                                                            {
                                                            case Direction::X:
                                                                for(size_t col = 0 ; col < path.normalized_recorded_points.size() ; ++col){
                                                                    point_in_itk_coordinates(0,col) =  _current_index;
                                                                    point_in_itk_coordinates(1,col) =  (int)std::round(path.normalized_recorded_points[col].fX * (volumetric_mask->dimension(Direction::Y) - 1));
                                                                    point_in_itk_coordinates(2,col) =  (int)std::round(path.normalized_recorded_points[col].fY * (volumetric_mask->dimension(Direction::Z) - 1));
                                                                }
                                                            break;
                                                            case Direction::Y:
                                                                for(size_t col = 0 ; col < path.normalized_recorded_points.size() ; ++col){
                                                                    point_in_itk_coordinates(0,col) =  (int)std::round(path.normalized_recorded_points[col].fX * (volumetric_mask->dimension(Direction::X) - 1));
                                                                    point_in_itk_coordinates(1,col) =  _current_index;
                                                                    point_in_itk_coordinates(2,col) =  (int)std::round(path.normalized_recorded_points[col].fY * (volumetric_mask->dimension(Direction::Z) - 1));
                                                                }
                                                            break;
                                                            case Direction::Z:
                                                                for(size_t col = 0 ; col < path.normalized_recorded_points.size() ; ++col){
                                                                    point_in_itk_coordinates(0,col) =  (int)std::round(path.normalized_recorded_points[col].fX * (volumetric_mask->dimension(Direction::X) - 1));
                                                                    point_in_itk_coordinates(1,col) =  (int)std::round(path.normalized_recorded_points[col].fY * (volumetric_mask->dimension(Direction::Y) - 1));
                                                                    point_in_itk_coordinates(2,col) = _current_index;
                                                                }
                                                            break;
                                                            }
                                                        },
                                                        [&](const curan::ui::Point &point)
                                                        {
                                                            point_in_itk_coordinates = Eigen::Matrix<double, 3,Eigen::Dynamic>::Zero(3,1);
                                                            switch (direction)
                                                            {
                                                            case Direction::X:
                                                            {
                                                                point_in_itk_coordinates(0,0) = _current_index;
                                                                point_in_itk_coordinates(1,0) = (int)std::round(point.normalized_point.fX * (volumetric_mask->dimension(Direction::Y) - 1));
                                                                point_in_itk_coordinates(2,0) = (int)std::round(point.normalized_point.fY * (volumetric_mask->dimension(Direction::Z) - 1));
                                                            }
                                                            break;
                                                            case Direction::Y:
                                                                point_in_itk_coordinates(0,0) = (int)std::round(point.normalized_point.fX * (volumetric_mask->dimension(Direction::X) - 1));
                                                                point_in_itk_coordinates(1,0) = _current_index;
                                                                point_in_itk_coordinates(2,0) = (int)std::round(point.normalized_point.fY * (volumetric_mask->dimension(Direction::Z) - 1));
                                                            break;
                                                            case Direction::Z:
                                                                point_in_itk_coordinates(0,0) = (int)std::round(point.normalized_point.fX * (volumetric_mask->dimension(Direction::X) - 1));
                                                                point_in_itk_coordinates(1,0) = (int)std::round(point.normalized_point.fY * (volumetric_mask->dimension(Direction::Y) - 1));
                                                                point_in_itk_coordinates(2,0) = _current_index;
                                                            break;
                                                            }
                                                        }},
                           *highlighted_and_pressed_stroke);
                if (pending_strokes_to_process.size() < 10)
                    pending_strokes_to_process.emplace_back(point_in_itk_coordinates, *highlighted_and_pressed_stroke, direction);
            }
        }

        if (!is_options && zoom_in && get_hightlight_color() == SkColorSetARGB(255, 125, 0, 0) && interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA))
            zoom_in.draw(canvas);

        slider_paint.setColor(slider_color);
        slider_paint.setStyle(SkPaint::kStroke_Style);

        SkRect dragable = SkRect::MakeXYWH(reserved_slider_space.x() + (reserved_slider_space.width() * (1 - dragable_percent_size)) * current_value, reserved_slider_space.y(), reserved_slider_space.width() * dragable_percent_size, reserved_slider_space.height());
        SkRect contained_squares = SkRect::MakeXYWH(reserved_slider_space.x(), reserved_slider_space.y(), reserved_slider_space.width() * dragable_percent_size, reserved_slider_space.height());
        canvas->drawRoundRect(reserved_slider_space, reserved_slider_space.height() / 2.0f, reserved_slider_space.height() / 2.0f, slider_paint);
        size_t increment_mask = 0;
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        volumetric_mask->for_each(direction, [&](const Mask &mask)
                                  {
        if(mask){
            slider_paint.setColor((increment_mask == _current_index) ? SK_ColorGREEN : hover_color);
            canvas->drawRoundRect(contained_squares, contained_squares.height() / 2.0f, contained_squares.height() / 2.0f, slider_paint);
        }
        contained_squares.offset(dragable.width(), 0);
        ++increment_mask; });
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

        canvas->drawRoundRect(dragable, reserved_slider_space.height() / 2.0f, reserved_slider_space.height() / 2.0f, slider_paint);
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

curan::ui::callablefunction SlidingPanel::call()
{
    auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
    {
        if(!compiled){
            throw std::runtime_error("must compile the SlidingPanel before call operations");
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
        auto [xpos, ypos] = interpreter.last_move();
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

        if (interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                curan::ui::InterpreterStatus::SCROLL_EVENT))
        {
            auto widget_rect = get_position();
            auto size = get_size();
            SkRect drawable = size;
            auto arg = std::get<curan::ui::Scroll>(sig);
            drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
            auto offsetx = (float)arg.xoffset / size.width();
            auto offsety = (float)arg.yoffset / size.width();
            auto current_val = get_current_value();
            current_val += (std::abs(offsetx) > std::abs(offsety)) ? offsetx : offsety;
            set_current_value(current_val);
            set_current_state(SliderStates::PRESSED);
            return true;
        }

        if(interpreter.check(curan::ui::InterpreterStatus::INSIDE_FIXED_AREA | 
                                curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT)){
            old_pressed_value = interpreter.last_move();
        }

        if (interpreter.check(curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED))
        {
            auto widget_rect = get_position();
            auto size = get_size();
            SkRect drawable = size;
            auto [xarg,yarg] = interpreter.last_move();
            auto [xarg_last,yarg_last] = old_pressed_value;
            auto current_val = get_current_value();
            drawable.offsetTo(widget_rect.centerX() - drawable.width() / 2.0f, widget_rect.centerY() - drawable.height() / 2.0f);
            auto offsetx = (float)(xarg-xarg_last) / size.width();
            set_current_value(offsetx+current_val);
            set_current_state(SliderStates::PRESSED);
            old_pressed_value = interpreter.last_move();
            return true;
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



        if(interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | curan::ui::InterpreterStatus::MOUSE_CLICKED_RIGHT_EVENT) ){
            set_current_state(SliderStates::PRESSED);
            //TODO: add circle describing options
            is_options = !is_options;
            return false;
        }


        if(is_options && interpreter.status() & ~curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED && 
            interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                curan::ui::InterpreterStatus::MOUSE_MOVE_EVENT | 
                                curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT) ){
            for(auto& opt : f_options){
                if(opt.absolute_location.contains(xpos,ypos)){
                    opt.is_clicked = false;
                    opt.is_hovering = true;
                }
            }
            return true;
        }

        if(is_options && interpreter.status() & ~curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED && 
            interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT) ){
                for(auto& opt : f_options){
                    if(opt.absolute_location.contains(xpos,ypos)){
                        opt.is_clicked = true;
                        opt.is_hovering = false;
                    }
                }
            return true;
        }

        for(auto& opt : f_options){
            if(opt.absolute_location.contains(xpos,ypos)){
                opt.is_clicked = false;
                opt.is_hovering = false;
            }
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

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkImageFileWriter.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "userinterface/Context.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/ConfigDraw.h"

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
using ImageType = itk::Image<PixelType, Dimension>;

int main() {
	try {
		using namespace curan::ui;
		IconResources resources{ CURAN_COPIED_RESOURCE_PATH"/images" };
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        
        using ImageReaderType = itk::ImageFileReader<itk::Image<double,3>>;

        std::printf("\nReading input volume...\n");
        auto fixedImageReader = ImageReaderType::New();
        fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha");
    
        // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
        auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double,3>, itk::Image<double,3>>::New();
        rescale->SetInput(fixedImageReader->GetOutput());
        rescale->SetOutputMinimum(0);
        rescale->SetOutputMaximum(255.0);
    
        auto castfilter = itk::CastImageFilter<itk::Image<double,3>, ImageType>::New();
        castfilter->SetInput(rescale->GetOutput());
        castfilter->Update();
        
        VolumetricMask vol{castfilter->GetOutput()};

        std::unique_ptr<SlidingPanel> image_display_x = SlidingPanel::make(resources, &vol, Direction::X);
        image_display_x->push_options({"coronal","axial","saggital"});
        std::unique_ptr<SlidingPanel> image_display_y = SlidingPanel::make(resources, &vol, Direction::Y);
        image_display_y->push_options({"coronal","axial","saggital"});
        std::unique_ptr<SlidingPanel> image_display_z = SlidingPanel::make(resources, &vol, Direction::Z);
        image_display_z->push_options({"coronal","axial","saggital"});

		auto container = curan::ui::Container::make(curan::ui::Container::ContainerType::LINEAR_CONTAINER,curan::ui::Container::Arrangement::HORIZONTAL);
		*container << std::move(image_display_x) << std::move(image_display_y) << std::move(image_display_z);

		auto page = curan::ui::Page{std::move(container),SK_ColorBLACK};
		page.update_page(viewer.get());

		curan::ui::ConfigDraw config_draw{ &page };

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated()) {
		    	page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();
            for(const auto& sig : signals)
                page.propagate_signal(sig, &config_draw);				

			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed" << e.what() << std::endl;
		return 1;
	}
}