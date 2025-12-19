#ifndef CURAN_COLOR_DICOM_DISPLAY_HEADER_FILE_
#define CURAN_COLOR_DICOM_DISPLAY_HEADER_FILE_

#include <functional>
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/ImageWrapper.h"
#include <unordered_map>
#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "itkExtractImageFilter.h"
#include "itkRGBAPixel.h"
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

using color_stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using color_sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;
using color_clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

enum ColorPathState{
    COLORSELECTPATH,
    COLORDRAWPATH,
    COLORDELETEPATH,
    COLORHIGHLIGHTPATH
};

class ColorDicomMask
{
    std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
    ColorDicomMask() = default;
    ColorDicomMask(const ColorDicomMask &m) = delete;
    ColorDicomMask &operator=(const ColorDicomMask &) = delete;

    template <typename... T>
    std::pair<std::unordered_map<size_t, curan::ui::Stroke>::iterator, bool> try_emplace(T &&...u)
    {
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

    inline void erase(const size_t key){
        recorded_strokes.erase(key);
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
        return recorded_strokes.size()>0;
    }

    std::optional<std::tuple<size_t,curan::ui::Stroke>> draw(SkCanvas *canvas, 
                                    const SkMatrix &inverse_homogenenous_transformation, 
                                    const SkMatrix &homogenenous_transformation, 
                                    const SkPoint &point,
                                    SkPaint &paint_stroke, 
                                    SkPaint &paint_square, 
                                    const SkFont &text_font, 
                                    bool is_pressed,
                                    ColorPathState current_path_state);
};

constexpr unsigned int ColorDimension = 3;

class ColorDicomVolumetricMask;
using colorpressedhighlighted_event = std::function<void(ColorDicomVolumetricMask*, curan::ui::ConfigDraw*, const directed_stroke&)>;

class ColorDicomVolumetricMask
{

    static size_t counter;
    static size_t identifier;

    using PixelType = itk::RGBAPixel<unsigned char>;
    using ImageType = itk::Image<PixelType, ColorDimension>;

    std::vector<ColorDicomMask> masks_x;
    std::vector<ColorDicomMask> masks_y;
    std::vector<ColorDicomMask> masks_z;

    std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>> three_dimensional_entities;

    ImageType::Pointer image;
public:
    std::list<colorpressedhighlighted_event> callbacks_pressedhighlighted;

    ColorDicomVolumetricMask(ImageType::Pointer volume);

    ColorDicomVolumetricMask(const ColorDicomVolumetricMask &m) = delete;
    ColorDicomVolumetricMask &operator=(const ColorDicomVolumetricMask &) = delete;

    inline void add_pressedhighlighted_call(colorpressedhighlighted_event &&call)
    {
        callbacks_pressedhighlighted.emplace_back(std::move(call));
    }

    inline bool filled(){
        return image.IsNotNull();
    }

    template<typename... T>
    bool try_replace(size_t previous_tag,const Direction &direction, const float &along_dimension, T &&...u)
    {
        assert(along_dimension >= 0 && along_dimension <= 1 && "the received size is not between 0 and 1");
        switch (direction)
        {
        case Direction::X:
        {
            auto _current_index_x = std::round(along_dimension * (masks_x.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_x[_current_index_x].try_emplace(previous_tag, std::forward<T>(u)...);
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
                                                                masks_y[_current_index_y].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
                                                                auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
                                                                masks_z[_current_index_z].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fX)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
            }
            if (erase)
                masks_x[_current_index_x].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        case Direction::Y:
        {
            auto _current_index_y = std::round(along_dimension * (masks_y.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_y[_current_index_y].try_emplace(previous_tag, std::forward<T>(u)...);
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
                                                                masks_x[_current_index_x].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(along_dimension, point.normalized_point.fY)});
                                                                auto _current_index_z = std::round(point.normalized_point.fY * (masks_z.size() - 1));
                                                                masks_z[_current_index_z].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
            }
            if (erase)
                masks_y[_current_index_y].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        case Direction::Z:
        {
            auto _current_index_z = std::round(along_dimension * (masks_z.size() - 1));
            auto [iterator_to_inserted_object,insertion_successeful] = masks_z[_current_index_z].try_emplace(previous_tag, std::forward<T>(u)...);
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
                                                                masks_x[_current_index_x].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(point.normalized_point.fY, along_dimension)});
                                                                auto _current_index_y = std::round(point.normalized_point.fY * (masks_y.size() - 1));
                                                                masks_y[_current_index_y].try_emplace(previous_tag, curan::ui::Point{SkPoint::Make(point.normalized_point.fX, along_dimension)});
                                                            }
                                                        }},
                           iterator_to_inserted_object->second);
            }
            if (erase)
                masks_z[_current_index_z].erase(iterator_to_inserted_object);
            return insertion_successeful;
        }
        default:
            throw std::runtime_error("incorrect mask direction selected");
        };
    }

    void remove_paths(const size_t key){
        for(auto& masks : masks_x)
            masks.erase(key);
        for(auto& masks : masks_y)
            masks.erase(key);
        for(auto& masks : masks_z)
            masks.erase(key);
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

    enum ColorPolicy{
        DISREGARD = 0,
        UPDATE_POINTS=2,
        UPDATE_GEOMETRIES = 4
    };

    inline void update_volume(ImageType::Pointer in_volume,int update_policy = ColorPolicy::DISREGARD,std::vector<size_t> identifiers = std::vector<size_t>{})
    {
        if(in_volume.IsNull())
            return;
        ImageType::RegionType inputRegion = in_volume->GetLargestPossibleRegion();
        std::vector<std::tuple<size_t,std::array<double,3>>> points_to_store;

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
                        local_normalized_index[0] = local_index[0]/(double)inputRegion.GetSize()[0];
                        local_normalized_index[1] = local_index[1]/(double)inputRegion.GetSize()[1];
                        local_normalized_index[2] = local_index[2]/(double)inputRegion.GetSize()[2];
                        bool are_there_constraints = identifiers.size();
                        if(are_there_constraints){
                            for(auto innerkey : identifiers)
                                if(key == innerkey)
                                    points_to_store.push_back(std::make_tuple(key,local_normalized_index));
                        } else {
                            points_to_store.push_back(std::make_tuple(key,local_normalized_index));
                        }      
                        }},stroke);
                }
            ++increment;
			}
		} 
        //std::printf("adress in_volume %llu address old_image %llu\n",(size_t)in_volume.get(),(size_t)image.get());   
        image = in_volume;
        masks_x = std::vector<ColorDicomMask>(inputRegion.GetSize()[Direction::X]);
        masks_y = std::vector<ColorDicomMask>(inputRegion.GetSize()[Direction::Y]);
        masks_z = std::vector<ColorDicomMask>(inputRegion.GetSize()[Direction::Z]);
        SkMatrix mat;
        mat.setIdentity();
        for(auto& [tag,point] : points_to_store)
            try_replace(tag,Direction::X,point[0],curan::ui::Point{SkPoint::Make(point[1],point[2]), mat});

        if(!(update_policy & UPDATE_GEOMETRIES))
            three_dimensional_entities = std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>>{};
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

    inline ColorDicomMask & current_mask(const Direction &direction, const size_t &along_dimension)
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

constexpr size_t color_size_of_slider_in_height = 30;
constexpr size_t color_buffer_around_panel = 8;

class ColorDicomViewer;
using coloroptionsselection_event = std::function<void(ColorDicomViewer*, curan::ui::ConfigDraw*, size_t selected_option)>;
using colorcustom_step = std::function<void(SkCanvas*, SkRect, SkRect)>;

class ColorDicomViewer final : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<ColorDicomViewer>
{
public:
    using PixelType = itk::RGBAPixel<unsigned char>;
    using ImageType = itk::Image<PixelType, ColorDimension>;
    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;

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
        ImageType::Pointer physical_image;
        double width_spacing = 1;
        double height_spacing = 1;
    };

private:
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

    ColorDicomVolumetricMask *volumetric_mask = nullptr;
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
    /*
    This is a client side image that is drawn after the backgroud image (or you could call it the clip between the volume and the viweing plane) is rendered. 
    What is the purpose? Sometimes we want to render two images that belong 
    to the same physical entity, e.g., a medical volume, hence we can provide a customized call for the client to 
    append their second image. What information does the client need to crop the images? Well we can provide them with
    a couple of geometric related parameters
    */
    std::optional<colorcustom_step> custom_drawing_call = std::nullopt;

    bool is_pressed = false;
    ColorPathState current_path_state = ColorPathState::COLORSELECTPATH;
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
    
    std::list<coloroptionsselection_event> callbacks_optionsselection;

    struct CurrentLocationDicom{
        ImageType::IndexType image_coordinates;
        ImageType::PointType world_coordinates;
        std::array<double,3> value;
    };

    std::vector<ImageWrapper> state_display;

    CurrentLocationDicom current_mouse_location;

    curan::ui::SignalInterpreter interpreter;

    void query_if_required(bool force_update);

    image_info extract_slice_from_volume(size_t index);

    ColorDicomViewer(curan::ui::IconResources &other, ColorDicomVolumetricMask *mask, Direction in_direction);

    void insert_in_map(const curan::ui::PointCollection &future_stroke);

public:
    static std::unique_ptr<ColorDicomViewer> make(curan::ui::IconResources &other, ColorDicomVolumetricMask *mask, Direction in_direction);

    ~ColorDicomViewer();

    void compile() override;

	ColorDicomViewer& update_custom_drawingcall(colorcustom_step call);

	ColorDicomViewer& clear_custom_drawingcall();

    std::optional<colorcustom_step> get_custom_drawingcall();

    inline ColorDicomViewer & add_overlay_processor(coloroptionsselection_event event_processor){
        callbacks_optionsselection.push_back(event_processor);
        return *(this);
    }

    void update_volume(ColorDicomVolumetricMask *mask, Direction in_direction);

    inline Direction get_direction() {
        std::lock_guard<std::mutex> g{get_mutex()};
        return direction;
    }
    
    void framebuffer_resize(const SkRect &new_page_size) override;
    void internal_framebuffer_recomputation();

    inline void change_zoom(){
        if (zoom_in)
            zoom_in.deactivate();
        else
            zoom_in.activate();
    }

    inline void change_path_state(ColorPathState state){
        if (!current_stroke.empty()){
            insert_in_map(current_stroke);
            current_stroke.clear();
        }

        if(COLORHIGHLIGHTPATH == state){
            if(COLORHIGHLIGHTPATH == current_path_state)
                current_path_state= COLORSELECTPATH;
            else 
                current_path_state = COLORHIGHLIGHTPATH;
            return;
        }

        if(COLORDELETEPATH == state){
            if(COLORDELETEPATH == current_path_state)
                current_path_state= COLORSELECTPATH;
            else 
                current_path_state = COLORDELETEPATH;
            return;
        }

        if(COLORDRAWPATH == state){
            if(COLORDRAWPATH == current_path_state)
                current_path_state= COLORSELECTPATH;
            else 
                current_path_state = COLORDRAWPATH;
            return;
        }
    }

    inline ImageType::Pointer physical_viewed_image(){
        std::lock_guard<std::mutex> g{get_mutex()};
        return  background.physical_image;
    }

    inline ColorDicomViewer &trigger(float in_current_value)
    {
        value_pressed = in_current_value;
        return *(this);
    }

    inline ColorDicomViewer & push_options(std::vector<std::string> options)
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

    inline ColorDicomViewer &set_current_value(float in_current_value)
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

    inline ColorDicomViewer &set_hover_color(SkColor color)
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

    inline ColorDicomViewer &set_waiting_color(SkColor new_waiting_color)
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

    inline ColorDicomViewer &set_click_color(SkColor new_click_color)
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

    inline ColorDicomViewer &set_hightlight_color(SkColor new_hightlight_color)
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

    inline ColorDicomViewer &set_slider_color(SkColor new_slider_color)
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

    inline ColorDicomViewer &set_current_state(SliderStates state)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        current_state = state;
        return *(this);
    }

    inline ColorDicomViewer& set_color_filter(sk_sp<SkColorFilter> filter){
        SkPaint paint;
        paint.setColorFilter(filter);
        std::lock_guard<std::mutex> g(get_mutex());
        paint_compliant_filtered_image = paint;
        return *(this);
    }

    inline ColorDicomViewer &set_options_hover_color(SkColor new_hightlight_color)
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

    inline ColorDicomViewer &set_options_waiting_color(SkColor new_hightlight_color)
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

    inline ColorDicomViewer &set_options_click_color(SkColor new_hightlight_color)
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

    inline ColorDicomViewer &set_options_text_color(SkColor new_hightlight_color)
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