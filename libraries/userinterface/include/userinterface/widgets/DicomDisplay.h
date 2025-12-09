#ifndef CURAN_DICOM_DISPLAY_HEADER_FILE_
#define CURAN_DICOM_DISPLAY_HEADER_FILE_

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
#include <map>
#include "utils/Overloading.h"
#include "utils/SafeQueue.h"
#include "geometry/Polyheadra.h"

namespace curan{
namespace ui{

using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;
using clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

enum MaskUsed
{
    CLEAN = 0,
    DIRTY
};

class DicomMask
{
    MaskUsed _mask_flag;
    std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
    DicomMask() : _mask_flag{MaskUsed::CLEAN} {}
    DicomMask(const DicomMask &m) = delete;
    DicomMask &operator=(const DicomMask &) = delete;

    template <typename... T>
    std::pair<std::unordered_map<size_t, curan::ui::Stroke>::iterator, bool> try_emplace(T &&...u)
    {
        _mask_flag = MaskUsed::DIRTY;
        return recorded_strokes.try_emplace(std::forward<T>(u)...);
    }

    template <typename... T>
    void for_each(T &&...u) const{
        std::for_each(recorded_strokes.begin(),recorded_strokes.end(),std::forward<T>(u)...);
    }

    inline const std::unordered_map<size_t, curan::ui::Stroke>& strokes() const {
        return recorded_strokes;
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

    std::optional<curan::ui::Stroke> draw(SkCanvas *canvas, 
                                    const SkMatrix &inverse_homogenenous_transformation, 
                                    const SkMatrix &homogenenous_transformation, 
                                    const SkPoint &point, bool is_highlighting, 
                                    SkPaint &paint_stroke, 
                                    SkPaint &paint_square, 
                                    const SkFont &text_font, 
                                    bool is_pressed);
};

constexpr unsigned int Dimension = 3;

class DicomVolumetricMask;
using pressedhighlighted_event = std::function<void(DicomVolumetricMask*, curan::ui::ConfigDraw*, const directed_stroke&)>;

class DicomVolumetricMask
{

    static size_t counter;
    static size_t identifier;

    using PixelType = unsigned char;
    using ImageType = itk::Image<PixelType, Dimension>;

    std::vector<DicomMask> masks_x;
    std::vector<DicomMask> masks_y;
    std::vector<DicomMask> masks_z;

    std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>> three_dimensional_entities;

    ImageType::Pointer image;
public:
    std::list<pressedhighlighted_event> callbacks_pressedhighlighted;

    DicomVolumetricMask(ImageType::Pointer volume);

    DicomVolumetricMask(const DicomVolumetricMask &m) = delete;
    DicomVolumetricMask &operator=(const DicomVolumetricMask &) = delete;

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

    enum Policy{
        DISREGARD = 0,
        UPDATE_POINTS,
        UPDATE_GEOMETRIES
    };

    inline void update_volume(ImageType::Pointer in_volume,int update_policy = Policy::DISREGARD,std::vector<size_t> identifiers = std::vector<size_t>{})
    {
        if(in_volume.IsNull())
            return;
        ImageType::RegionType inputRegion = in_volume->GetBufferedRegion();
        std::vector<std::array<double,3>> points_to_store;

		if((update_policy & UPDATE_POINTS) && image.IsNotNull()){
            auto old_size = image->GetLargestPossibleRegion().GetSize();
            size_t increment = 0;
			for(auto& mask : masks_x){ 
                auto strokes = mask.strokes();
                for(const auto& [key,stroke] : strokes){
                    std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path){},	//paths are basically meaningless
                                                            [&](const curan::ui::Point &point) { 
                        ImageType::IndexType local_index;
                        ImageType::PointType itk_point_in_world_coordinates;
                        local_index[0] =  increment;
                        local_index[1] =  (int)std::round(point.normalized_point.fX * (old_size[Direction::Y] - 1));
                        local_index[2] =  (int)std::round(point.normalized_point.fY * (old_size[Direction::Z] - 1));
                        image->TransformIndexToPhysicalPoint(local_index,itk_point_in_world_coordinates);
                        in_volume->TransformPhysicalPointToIndex(itk_point_in_world_coordinates,local_index);
                        std::array<double,3> local_normalized_index;
                        local_normalized_index[0] = local_index[0]/inputRegion.GetSize()[0];
                        local_normalized_index[1] = local_index[1]/inputRegion.GetSize()[1];
                        local_normalized_index[2] = local_index[2]/inputRegion.GetSize()[2];
                        bool isthere = identifiers.size() ? false : true;
                        for(auto innerkey : identifiers)
                            if(key == innerkey)
                                isthere = true;
                            if(isthere)
                                points_to_store.push_back(local_normalized_index);
                        }},stroke);
                }
			}
		} 
                
        if(! ((update_policy & UPDATE_GEOMETRIES) && image.IsNotNull()))
            three_dimensional_entities = std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>>{};

        image = in_volume;
        masks_x = std::vector<DicomMask>(inputRegion.GetSize()[Direction::X]);
        masks_y = std::vector<DicomMask>(inputRegion.GetSize()[Direction::Y]);
        masks_z = std::vector<DicomMask>(inputRegion.GetSize()[Direction::Z]);
        SkMatrix mat;
        mat.setIdentity();
        for(auto point : points_to_store)
            try_emplace(Direction::X,point[0],curan::ui::Point{SkPoint::Make(point[1],point[2]), mat});
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
    [[nodiscard]] std::optional<std::string> add_geometry(T&& geometry_to_add,SkColor color){
        ++identifier;
        std::string str_ident = "geometry"+std::to_string(identifier);
        auto retu = three_dimensional_entities.try_emplace(str_ident,std::make_tuple(std::forward<T>(geometry_to_add),color));
        return retu.second ? std::optional<std::string>{str_ident} : std::nullopt;
    }

    bool delete_geometry(std::string to_delete){
        bool deleted = false;
        for (auto it = three_dimensional_entities.begin(); it != three_dimensional_entities.end();){
            if (!(it->first.compare(to_delete))){
                deleted = true;
                it = three_dimensional_entities.erase(it);
                break;
            }
            else
                ++it;
        }
        return deleted;
    }

    inline const std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>>& geometries() const{
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

    inline DicomMask & current_mask(const Direction &direction, const size_t &along_dimension)
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

class DicomViewer;
using optionsselection_event = std::function<void(DicomViewer*, curan::ui::ConfigDraw*, size_t selected_option)>;

class DicomViewer final : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<DicomViewer>
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

    const double buffer_sideways = 30.0;

    DicomVolumetricMask *volumetric_mask = nullptr;
    ImageType* chached_pointer = nullptr;
    size_t cached_number_of_geometries = 0;
    double cached_sum_of_geometries = 0.0;
    
    std::vector<std::tuple<std::vector<SkPoint>,SkPath,SkColor>> cached_polyheader_intersections;
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

    struct CurrentLocationDicom{
        ImageType::IndexType image_coordinates;
        ImageType::PointType world_coordinates;
        double value;
    };
    
    CurrentLocationDicom current_mouse_location;

    curan::ui::SignalInterpreter interpreter;

    void query_if_required(bool force_update);

    image_info extract_slice_from_volume(size_t index);

    DicomViewer(curan::ui::IconResources &other, DicomVolumetricMask *mask, Direction in_direction);

    void insert_in_map(const curan::ui::PointCollection &future_stroke);

public:
    static std::unique_ptr<DicomViewer> make(curan::ui::IconResources &other, DicomVolumetricMask *mask, Direction in_direction);

    ~DicomViewer();

    void compile() override;

    inline DicomViewer & add_overlay_processor(optionsselection_event event_processor){
        callbacks_optionsselection.push_back(event_processor);
        return *(this);
    }

    void update_volume(DicomVolumetricMask *mask, Direction in_direction);

    void framebuffer_resize(const SkRect &new_page_size) override;
    void internal_framebuffer_recomputation();

    inline void change_zoom(){
        if (zoom_in)
            zoom_in.deactivate();
        else
            zoom_in.activate();
    }

    inline void change_path_selection(){
        is_highlighting = !is_highlighting;
        if (!current_stroke.empty()){
            insert_in_map(current_stroke);
            current_stroke.clear();
        }
    }

    inline DicomViewer &trigger(float in_current_value)
    {
        value_pressed = in_current_value;
        return *(this);
    }

    inline DicomViewer & push_options(std::vector<std::string> options)
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

    inline DicomViewer &set_current_value(float in_current_value)
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

    inline DicomViewer &set_hover_color(SkColor color)
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

    inline DicomViewer &set_waiting_color(SkColor new_waiting_color)
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

    inline DicomViewer &set_click_color(SkColor new_click_color)
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

    inline DicomViewer &set_hightlight_color(SkColor new_hightlight_color)
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

    inline DicomViewer &set_slider_color(SkColor new_slider_color)
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

    inline DicomViewer &set_current_state(SliderStates state)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        current_state = state;
        return *(this);
    }

    inline DicomViewer& set_color_filter(sk_sp<SkColorFilter> filter){
        SkPaint paint;
        paint.setColorFilter(filter);
        std::lock_guard<std::mutex> g(get_mutex());
        paint_compliant_filtered_image = paint;
        return *(this);
    }

    inline DicomViewer &set_options_hover_color(SkColor new_hightlight_color)
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

    inline DicomViewer &set_options_waiting_color(SkColor new_hightlight_color)
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

    inline DicomViewer &set_options_click_color(SkColor new_hightlight_color)
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

    inline DicomViewer &set_options_text_color(SkColor new_hightlight_color)
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

}
}

#endif 