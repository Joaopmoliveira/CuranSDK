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
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"
#include "geometry/Intersection.h"
#include "itkRegionOfInterestImageFilter.h"
#include <format>

namespace curan{
namespace ui{

using stroke_added_callback = std::function<curan::ui::Stroke(void)>;
using sliding_panel_callback = std::function<std::optional<curan::ui::ImageWrapper>(size_t slider_value)>;
using clicked_highlighted_stroke_callback = std::function<curan::ui::Stroke(void)>;

enum PathState{
    SELECTPATH,
    DRAWPATH,
    DELETEPATH,
    HIGHLIGHTPATH
};

class DicomMask
{
    std::unordered_map<size_t, curan::ui::Stroke> recorded_strokes;

public:
    DicomMask() = default;
    DicomMask(const DicomMask &m) = delete;
    DicomMask &operator=(const DicomMask &) = delete;

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

    void container_resized(const SkMatrix &inverse_homogenenous_transformation)
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

    inline operator bool() const
    {
        return recorded_strokes.size()>0;
    }

    std::optional<std::tuple<size_t,curan::ui::Stroke>> draw(
        SkCanvas *canvas, 
        const SkMatrix &inverse_homogenenous_transformation, 
        const SkMatrix &homogenenous_transformation, 
        const SkPoint &point, 
        SkPaint &paint_stroke, 
        SkPaint &paint_square, 
        const SkFont &text_font, 
        bool is_pressed, 
        PathState current_path_state)
    {    
        if (current_path_state == HIGHLIGHTPATH || current_path_state == DELETEPATH)
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
                    return std::make_tuple(minimum_index->first,minimum_index->second);
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
};

constexpr unsigned int Dimension = 3;

template<typename PixelType>
class DicomVolumetricMask;
template<typename PixelType>
using pressedhighlighted_event = std::function<void(DicomVolumetricMask<PixelType>*, curan::ui::ConfigDraw*, const directed_stroke&)>;

template<typename PixelType>
class DicomVolumetricMask
{

    static size_t counter;
    static size_t identifier;
    using ImageType = itk::Image<PixelType, Dimension>;
    using PressedEvent = pressedhighlighted_event<DicomVolumetricMask<PixelType>>;
    std::vector<DicomMask> masks_x;
    std::vector<DicomMask> masks_y;
    std::vector<DicomMask> masks_z;

    std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>> three_dimensional_entities;

    typename ImageType::Pointer image;
public:
    std::list<PressedEvent> callbacks_pressedhighlighted;

    DicomVolumetricMask(ImageType::Pointer volume) : image{volume}
    {
        update_volume(volume);
    }

    DicomVolumetricMask(const DicomVolumetricMask &m) = delete;
    DicomVolumetricMask &operator=(const DicomVolumetricMask &) = delete;

    inline void add_pressedhighlighted_call(PressedEvent &&call)
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

    enum Policy{
        DISREGARD = 0,
        UPDATE_POINTS=2,
        UPDATE_GEOMETRIES = 4
    };

    inline void update_volume(typename ImageType::Pointer in_volume,int update_policy = Policy::DISREGARD,std::vector<size_t> identifiers = std::vector<size_t>{})
    {
        if(in_volume.IsNull())
            return;
        typename ImageType::RegionType inputRegion = in_volume->GetLargestPossibleRegion();
        std::vector<std::tuple<size_t,std::array<double,3>>> points_to_store;

		if((update_policy & UPDATE_POINTS) && image.IsNotNull()){
            auto old_size = image->GetLargestPossibleRegion().GetSize();
            size_t increment = 0;
			for(auto& mask : masks_x){ 
                auto strokes = mask.strokes();
                for(const auto& [key,stroke] : strokes){
                    std::visit(curan::utilities::overloaded{[&](const curan::ui::Path &path){},	//paths are basically meaningless
                                                            [&](const curan::ui::Point &point) { 
                        typename ImageType::IndexType local_index;
                        typename ImageType::PointType itk_point_in_world_coordinates;
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
        masks_x = std::vector<DicomMask>(inputRegion.GetSize()[Direction::X]);
        masks_y = std::vector<DicomMask>(inputRegion.GetSize()[Direction::Y]);
        masks_z = std::vector<DicomMask>(inputRegion.GetSize()[Direction::Z]);
        SkMatrix mat;
        mat.setIdentity();
        for(auto& [tag,point] : points_to_store)
            try_replace(tag,Direction::X,point[0],curan::ui::Point{SkPoint::Make(point[1],point[2]), mat});

        if(!(update_policy & UPDATE_GEOMETRIES))
            three_dimensional_entities = std::map<std::string,std::tuple<curan::geometry::PolyHeadra,SkColor>>{};
    }

    inline typename ImageType::Pointer get_volume(){
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

template<typename PixelType>
class DicomViewer;
template<typename PixelType>
using optionsselection_event = std::function<void(DicomViewer<PixelType>*, curan::ui::ConfigDraw*, size_t selected_option)>;
using custom_step = std::function<void(SkCanvas*, SkRect, SkRect)>;

template<typename PixelType>
class DicomViewer final : public curan::ui::Drawable, public curan::utilities::Lockable, public curan::ui::SignalProcessor<typename DicomViewer<PixelType>>
{
    using ImageType = itk::Image<PixelType, Dimension>;
    using ExtractFilterType = itk::ExtractImageFilter<ImageType, ImageType>;
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
        typename ImageType::Pointer physical_image;
        double width_spacing = 1;
        double height_spacing = 1;
    };

private:
    typename ExtractFilterType::Pointer extract_filter;

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

    typename DicomVolumetricMask<PixelType> *volumetric_mask = nullptr;
    typename ImageType* chached_pointer = nullptr;
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
    std::optional<custom_step> custom_drawing_call = std::nullopt;

    bool is_pressed = false;
    PathState current_path_state = PathState::SELECTPATH;
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

    using SelectEvent = optionsselection_event<PixelType>;

    std::list<SelectEvent> callbacks_optionsselection;

    struct CurrentLocationDicom{
        typename ImageType::IndexType image_coordinates;
        typename ImageType::PointType world_coordinates;
        double value;
    };

    std::vector<ImageWrapper> state_display;

    CurrentLocationDicom current_mouse_location;

    curan::ui::SignalInterpreter interpreter;

    void query_if_required(bool force_update)
    { //TODO
        size_t previous = _current_index;
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        if (!volumetric_mask->filled())
            return;
        _current_index = std::round(current_value * (volumetric_mask->dimension(direction) - 1));

        if (force_update)
        {
            background = extract_slice_from_volume(_current_index);
            internal_framebuffer_recomputation();
        }
        else if (previous != _current_index){
            background = extract_slice_from_volume(_current_index);
            internal_framebuffer_recomputation();
        }
        previous = _current_index;
    }

    image_info extract_slice_from_volume(size_t index)
    {
        image_info info;
        assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
        
        // Define the filter type (Input and Output types are the same 3D image)
        using ROIFilterType = itk::RegionOfInterestImageFilter<ImageType,ImageType>;

        typename ROIFilterType::Pointer roi_filter = ROIFilterType::New();
        roi_filter->SetInput(volumetric_mask->get_volume());

        // 1. Setup the Region
        // Make sure to use LargestPossibleRegion or UpdateOutputInformation first
        volumetric_mask->get_volume()->UpdateOutputInformation(); 
        typename ImageType::RegionType inputRegion = volumetric_mask->get_volume()->GetLargestPossibleRegion();

        typename ImageType::SizeType size = inputRegion.GetSize();
        size[direction] = 1; // Slice thickness of 1
        typename ImageType::SpacingType spacing = volumetric_mask->get_volume()->GetSpacing();
        typename ImageType::IndexType start = inputRegion.GetIndex();
        start[direction] = index; // The slice index you want

        typename ImageType::RegionType desiredRegion;
        desiredRegion.SetSize(size);
        desiredRegion.SetIndex(start);
    

        // 2. Set the Region of Interest
        roi_filter->SetRegionOfInterest(desiredRegion);
        roi_filter->Update();
        
       /*
        extract_filter = ExtractFilterType::New();
        extract_filter->SetDirectionCollapseToSubmatrix();
        extract_filter->SetInput(volumetric_mask->get_volume());
    
        ImageType::RegionType inputRegion = volumetric_mask->get_volume()->GetLargestPossibleRegion();
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
        extract_filter->Update();
        */
        typename ImageType::Pointer pointer_to_block_of_memory = roi_filter->GetOutput();
    
        info.physical_image = pointer_to_block_of_memory;
        typename ImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
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
        cached_polyheader_intersections = std::vector<std::tuple<std::vector<SkPoint>, SkPath,SkColor>>{};
        for (const auto &[key,geomdata] : volumetric_mask->geometries())
        {
            const auto &[geomtry_to_intersect,color] = geomdata;
            std::vector<SkPoint> points_in_path;

            auto voldirection = volumetric_mask->get_volume()->GetDirection();
            auto size = volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize();
            typename ImageType::PointType vol_slice_center;
            typename ImageType::IndexType index{{(size_t)(size[0]*0.5), (size_t)(size[1]*0.5), (size_t)(size[2]*0.5)}};
            index[direction] = size[direction]*current_value;
            volumetric_mask->get_volume()->TransformIndexToPhysicalPoint(index, vol_slice_center);
            Eigen::Matrix<double, 3, 1> normal{voldirection(0,direction),voldirection(1,direction),voldirection(2,direction)};
            Eigen::Matrix<double, 3, 1> origin{vol_slice_center[0], vol_slice_center[1], vol_slice_center[2]};
    
            auto possible_cliped_polygon = curan::geometry::clip_with_plane(geomtry_to_intersect, normal, origin);
            if (!possible_cliped_polygon)
            {
                continue;
            }
    
            if ((*possible_cliped_polygon).cols() == 0)
            {
                continue;
            }
            Eigen::Matrix<double,3,Eigen::Dynamic> staged_copy = *possible_cliped_polygon;
            for(size_t i = 0; i< staged_copy.cols(); ++i){
                typename ImageType::PointType physical_intersection_point{{staged_copy.col(i)[0],staged_copy.col(i)[1],staged_copy.col(i)[2]}};
                typename ImageType::IndexType index;    
                volumetric_mask->get_volume()->TransformPhysicalPointToIndex(physical_intersection_point,index);
                auto size = volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize();
                staged_copy.col(i)[0] = index[0]/(double)size[0];
                staged_copy.col(i)[1] = index[1]/(double)size[1];
                staged_copy.col(i)[2] = index[2]/(double)size[2];
            }
            possible_cliped_polygon = staged_copy;
    
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
            } // the points here are in normalized image units, now we need to ask skia to transform they to whaver coordinates they should be rendered on screen
    
            std::vector<SkPoint> transformed_points = points_in_path;
    
            inverse_homogenenous_transformation.mapPoints(transformed_points.data(), transformed_points.size());
            SkPath polygon_intersection;
            polygon_intersection.moveTo(transformed_points.front());
            for (const auto &point : transformed_points)
                polygon_intersection.lineTo(point);
            polygon_intersection.close();
    
            cached_polyheader_intersections.push_back(std::make_tuple(points_in_path, polygon_intersection,color));
        }
    
        return info;
    }

    DicomViewer(curan::ui::IconResources &other, 
                            typename DicomVolumetricMask<PixelType> *volume_mask, 
                            Direction in_direction) : volumetric_mask{volume_mask},
                                                        chached_pointer{volume_mask->get_volume().GetPointer()},
                                                        cached_number_of_geometries{volume_mask->geometries().size()}, 
                                                        system_icons{other}
    {
        cached_sum_of_geometries = 0.0;
        for(const auto& [key,geomdata] : volume_mask->geometries()){
            const auto& [geom,color] = geomdata;
            for(const auto& ver : geom.geometry.vertices){
                cached_sum_of_geometries += (double)ver[0] + (double)ver[1] + (double)ver[2];
            }
        }

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

    void insert_in_map(const curan::ui::PointCollection &future_stroke)
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

public:
    static std::unique_ptr<DicomViewer<PixelType>> make(curan::ui::IconResources &other, DicomVolumetricMask<PixelType> *volume_mask, Direction in_direction)
    {
        std::unique_ptr<DicomViewer<PixelType>> button = std::unique_ptr<DicomViewer<PixelType>>(new DicomViewer<PixelType>{other, volume_mask, in_direction});
        return button;
    }

    ~DicomViewer()
    {
    }

    void compile() override
    {
        if(compiled){
            throw std::runtime_error("cannot compile twice");
        }
        if (system_icons.is_loaded()) {
            auto image = system_icons.get_icon("normalselection.png");
            if(!image)
                throw std::runtime_error("failed to query identifier");
            state_display.push_back(*image); 
            image = system_icons.get_icon("drawpaths.png");
            if(!image)
                throw std::runtime_error("failed to query identifier"); 
            state_display.push_back(*image);
            image = system_icons.get_icon("deletepaths.png");
            if(!image)
                throw std::runtime_error("failed to query identifier"); 
            state_display.push_back(*image);
            image = system_icons.get_icon("highlightpaths.png");
            if(!image)
                throw std::runtime_error("failed to query identifier"); 
            state_display.push_back(*image);
        }
        compiled = true;
    }

	DicomViewer& update_custom_drawingcall(custom_step call) {
        std::lock_guard<std::mutex> g{ get_mutex() };
        custom_drawing_call = call;
        return *(this);
    }

	DicomViewer& clear_custom_drawingcall() {
        std::lock_guard<std::mutex> g{ get_mutex() };
        custom_drawing_call = std::nullopt;
        return *(this);
    }

    std::optional<custom_step> get_custom_drawingcall() {
        std::lock_guard<std::mutex> g{ get_mutex() };
        return custom_drawing_call;
    }

    inline DicomViewer<PixelType> & add_overlay_processor(SelectEvent event_processor){
        callbacks_optionsselection.push_back(event_processor);
        return *(this);
    }

    void update_volume(DicomVolumetricMask<PixelType> *volume_mask, Direction in_direction)
    {
        std::lock_guard<std::mutex> g{get_mutex()};
        assert(volume_mask != nullptr && "volumetric mask must be different from nullptr");
        if (!volume_mask->filled())
            return;
        direction = in_direction;
        _current_index = std::round(current_value * (volumetric_mask->dimension(direction) - 1));
        volumetric_mask = volume_mask;
        dragable_percent_size = 1.0 / volumetric_mask->dimension(direction);
        query_if_required(true);
    }

    inline Direction get_direction() {
        std::lock_guard<std::mutex> g{get_mutex()};
        return direction;
    }
    
    void framebuffer_resize(const SkRect &new_page_size) override
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
        volumetric_mask->for_each(direction, [&](DicomMask &mask)
                                  { mask.container_resized(inverse_homogenenous_transformation); });
    
        for (auto &[normalized_path, cached_path,color] : cached_polyheader_intersections)
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

        if(max_items_per_line > 3.0){
            number_per_line = 3;
        }
    
        double in_line_spacing  = (reserved_drawing_space.width()-2.0*buffer_sideways-(number_per_line-1)*max_width)/number_per_line;
        double x_location = reserved_drawing_space.fLeft+buffer_sideways;
        double y_location = reserved_drawing_space.fTop+buffer_sideways;
    
        size_t row_location_index = 0;
    
        for(auto& opt : f_options){
            opt.absolute_location = SkRect::MakeLTRB(x_location,y_location,x_location+max_width,y_location+max_height);    
            x_location += max_width+in_line_spacing;
            ++row_location_index;
            if(row_location_index >= number_per_line){
                row_location_index = 0;
                y_location += max_height+buffer_sideways;
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
        volumetric_mask->for_each(direction, [&](DicomMask &mask)
                                  { mask.container_resized(inverse_homogenenous_transformation); });
    
        for (auto &[normalized_path, cached_path,color] : cached_polyheader_intersections)
        {
            std::vector<SkPoint> transformed_points = normalized_path;
            inverse_homogenenous_transformation.mapPoints(transformed_points.data(), transformed_points.size());
            cached_path.reset();
            cached_path.moveTo(transformed_points.front());
            for (const auto &point : transformed_points)
                cached_path.lineTo(point);
            cached_path.close();
        }

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

        if(max_items_per_line > 3.0){
            number_per_line = 3;
        }
    
        double in_line_spacing  = (reserved_drawing_space.width()-2.0*buffer_sideways-(number_per_line-1)*max_width)/number_per_line;
        double x_location = reserved_drawing_space.fLeft+buffer_sideways;
        double y_location = reserved_drawing_space.fTop+buffer_sideways;
    
        size_t row_location_index = 0;
    
        for(auto& opt : f_options){
            opt.absolute_location = SkRect::MakeLTRB(x_location,y_location,x_location+max_width,y_location+max_height);    
            x_location += max_width+in_line_spacing;
            ++row_location_index;
            if(row_location_index >= number_per_line){
                row_location_index = 0;
                y_location += max_height+buffer_sideways;
                x_location = reserved_drawing_space.fLeft+buffer_sideways;
            }
        }  
        return;
    }

    inline void change_zoom(){
        if (zoom_in)
            zoom_in.deactivate();
        else
            zoom_in.activate();
    }

    inline void change_path_state(PathState state){
        if (!current_stroke.empty()){
            insert_in_map(current_stroke);
            current_stroke.clear();
        }

        if(HIGHLIGHTPATH == state){
            if(HIGHLIGHTPATH == current_path_state)
                current_path_state= SELECTPATH;
            else 
                current_path_state = HIGHLIGHTPATH;
            return;
        }

        if(DELETEPATH == state){
            if(DELETEPATH == current_path_state)
                current_path_state= SELECTPATH;
            else 
                current_path_state = DELETEPATH;
            return;
        }

        if(DRAWPATH == state){
            if(DRAWPATH == current_path_state)
                current_path_state= SELECTPATH;
            else 
                current_path_state = DRAWPATH;
            return;
        }
    }

    inline ImageType::Pointer physical_viewed_image(){
        std::lock_guard<std::mutex> g{get_mutex()};
        return  background.physical_image;
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

    curan::ui::drawablefunction draw() override
    {
        auto lamb = [this](SkCanvas *canvas)
        {
        if(!compiled){
            throw std::runtime_error("must compile the DicomViewer before drawing operations");
        }
    
        if (!volumetric_mask->filled())
            return;
        auto widget_rect = get_position();

        if(chached_pointer!=volumetric_mask->get_volume().GetPointer()) {
            chached_pointer = volumetric_mask->get_volume().GetPointer();
            cached_number_of_geometries = volumetric_mask->geometries().size();
            cached_sum_of_geometries = 0.0;
            for(const auto& [key,geomdata] : volumetric_mask->geometries()){
                const auto& [geom,color] = geomdata;
                for(const auto& ver : geom.geometry.vertices)
                    cached_sum_of_geometries += (double)ver[0] + (double)ver[1] + (double)ver[2];
            }
            query_if_required(true);
            framebuffer_resize(SkRect::MakeWH(0,0));
        }  else if(volumetric_mask->geometries().size()!=cached_number_of_geometries) {
            cached_number_of_geometries = volumetric_mask->geometries().size();
            cached_sum_of_geometries = 0.0;
            for(const auto& [key,geomdata] : volumetric_mask->geometries()){
                const auto& [geom,color] = geomdata;
                for(const auto& ver : geom.geometry.vertices)
                    cached_sum_of_geometries += (double)ver[0] + (double)ver[1] + (double)ver[2];
            }
            query_if_required(true);
            framebuffer_resize(SkRect::MakeWH(0,0));
        } else {
            double sum_of_geometries = 0.0;
            for(const auto& [key,geomdata] : volumetric_mask->geometries()){
                const auto& [geom,color] = geomdata;
                for(const auto& ver : geom.geometry.vertices)
                    sum_of_geometries += (double)ver[0] + (double)ver[1] + (double)ver[2];
            }
            if(std::abs(sum_of_geometries-cached_sum_of_geometries) > 0.000001){
                cached_sum_of_geometries = sum_of_geometries;
                query_if_required(true);
                framebuffer_resize(SkRect::MakeWH(0,0));
            }
        }

        bool is_panel_selected = false;
        {
            SkAutoCanvasRestore restore{canvas, true};
            canvas->clipRect(get_position());
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
                auto call = get_custom_drawingcall();
                if(call)
                    (*call)(canvas,background_rect,widget_rect);
            } 
            canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.transformed_recorded_points.size(), current_stroke.transformed_recorded_points.data(), paint_stroke);
            {
                is_panel_selected = get_hightlight_color() == SkColorSetARGB(255, 125, 0, 0);
                // TODO: here I need to convert the coordinates of the last press mouse into the coordinates of the volume to render on screen
                SkPaint paint_cached_paths;
                paint_cached_paths.setAntiAlias(true);
                paint_cached_paths.setStyle(SkPaint::kStrokeAndFill_Style);
                paint_cached_paths.setColor(SkColorSetARGB(0x0F, 0xFF, 0x00, 0x00));
                auto paint_cached_paths_outline = paint_cached_paths;
                paint_cached_paths_outline.setColor(SkColorSetARGB(0xFF, 0xFF, 0x00, 0x00));
                paint_cached_paths_outline.setStyle(SkPaint::kStroke_Style);
                std::lock_guard<std::mutex> g{get_mutex()};
    
                for (const auto &[normalized_path, cached_path,color] : cached_polyheader_intersections)
                {
                    //canvas->drawPath(cached_path, paint_cached_paths);
                    paint_cached_paths_outline.setColor(color);
                    canvas->drawPath(cached_path, paint_cached_paths_outline);
                }
    
                assert(volumetric_mask != nullptr && "volumetric mask must be different from nullptr");
                if(is_options) is_pressed = false;
                auto highlighted_and_pressed_stroke = volumetric_mask->current_mask(direction, _current_index).draw(canvas, inverse_homogenenous_transformation, homogenenous_transformation, zoom_in.get_coordinates(), paint_stroke, paint_square, text_font, is_pressed,current_path_state);
                if (highlighted_and_pressed_stroke)
                {
                    auto& [key,stroke] = *highlighted_and_pressed_stroke;
                    is_pressed = false;
                    if(current_path_state == DELETEPATH)
                        volumetric_mask->remove_paths(key);
                    else {
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
                                stroke);
                        if (pending_strokes_to_process.size() < 10)
                            pending_strokes_to_process.emplace_back(point_in_itk_coordinates, stroke, direction);
                    }
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
    
            if(is_panel_selected){
                std::string pixel_value;
                if(current_mouse_location.value >= 0.0){
                    pixel_value = "Pixel value: " + std::format("{:.2f}", current_mouse_location.value);
                } else 
                    pixel_value = "Pixel value: Out of range";
                
                std::string image_coordinates = "Image coordinates: [" + std::to_string((int)current_mouse_location.image_coordinates[0]) + " , " + std::to_string((int)current_mouse_location.image_coordinates[1]) + " , " + std::to_string((int)current_mouse_location.image_coordinates[2]) + "]";
                std::string frame_coordinates = "Frame coordinates: [" + std::to_string((int)current_mouse_location.world_coordinates[0]) + " , " + std::to_string((int)current_mouse_location.world_coordinates[1]) + " , " + std::to_string((int)current_mouse_location.world_coordinates[2]) + "]";
    
                options_paint.setColor(SK_ColorWHITE);
                canvas->drawSimpleText(pixel_value.data(),pixel_value.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-3.0*font_size,text_font,options_paint);    
                canvas->drawSimpleText(image_coordinates.data(),image_coordinates.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-2.0*font_size,text_font,options_paint);   
                canvas->drawSimpleText(frame_coordinates.data(),frame_coordinates.size(),SkTextEncoding::kUTF8,reserved_slider_space.fLeft+buffer_sideways,reserved_slider_space.fTop-buffer_sideways-font_size,text_font,options_paint);   
            }
    
            volumetric_mask->for_each(direction, [&](const DicomMask &mask)
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

            const int offset_from_border = contained_squares.height()+10;
            const int icon_size = 50;
            SkRect current_selected_image_rectangle = SkRect::MakeLTRB(widget_rect.fRight-offset_from_border-icon_size,widget_rect.fBottom-offset_from_border-icon_size,widget_rect.fRight-offset_from_border,widget_rect.fBottom-offset_from_border);
            SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{ 1.0f / 3.0f, 1.0f / 3.0f });
            switch(current_path_state){
                case SELECTPATH:
                    canvas->drawImageRect(state_display[SELECTPATH].image, current_selected_image_rectangle, opt);
                    break;
                case DRAWPATH:
                    canvas->drawImageRect(state_display[DRAWPATH].image, current_selected_image_rectangle, opt);
                    break;
                case DELETEPATH: 
                    canvas->drawImageRect(state_display[DELETEPATH].image, current_selected_image_rectangle, opt);
                    break;
                case HIGHLIGHTPATH:
                    canvas->drawImageRect(state_display[HIGHLIGHTPATH].image, current_selected_image_rectangle, opt);
                    break;
                default:
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

    curan::ui::callablefunction call() override
    {
        auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
        {
            if(!compiled){
                throw std::runtime_error("must compile the DicomViewer before call operations");
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
    
            if(chached_pointer!=volumetric_mask->get_volume().GetPointer() || volumetric_mask->geometries().size()!=cached_number_of_geometries) {
                chached_pointer = volumetric_mask->get_volume().GetPointer();
                cached_number_of_geometries = volumetric_mask->geometries().size();
                cached_sum_of_geometries = 0.0;
                for(const auto& [key,geomdata] : volumetric_mask->geometries()){
                    const auto& [geom,color] = geomdata;
                    for(const auto& ver : geom.geometry.vertices)
                        cached_sum_of_geometries += (double)ver[0] + (double)ver[1] + (double)ver[2];
                }
                query_if_required(true);
                framebuffer_resize(SkRect::MakeWH(0,0));
            }
    
            interpreter.process(check_inside_allocated_area, check_inside_fixed_area, sig);
    
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
                switch (direction)
                {
                case Direction::X:
                    current_mouse_location.image_coordinates[0] =  _current_index;
                    current_mouse_location.image_coordinates[1] =  (int)std::round(point.fX * (volumetric_mask->dimension(Direction::Y) - 1));
                    current_mouse_location.image_coordinates[2] =  (int)std::round(point.fY * (volumetric_mask->dimension(Direction::Z) - 1));
                break;
                case Direction::Y:
                    current_mouse_location.image_coordinates[0] =  (int)std::round(point.fX * (volumetric_mask->dimension(Direction::X) - 1));
                    current_mouse_location.image_coordinates[1] =  _current_index;
                    current_mouse_location.image_coordinates[2] =  (int)std::round(point.fY * (volumetric_mask->dimension(Direction::Z) - 1));
                break;
                case Direction::Z:
                    current_mouse_location.image_coordinates[0] =  (int)std::round(point.fX * (volumetric_mask->dimension(Direction::X) - 1));
                    current_mouse_location.image_coordinates[1] =  (int)std::round(point.fY * (volumetric_mask->dimension(Direction::Y) - 1));
                    current_mouse_location.image_coordinates[2] = _current_index;
                break;
                }
                volumetric_mask->get_volume()->TransformIndexToPhysicalPoint(current_mouse_location.image_coordinates,current_mouse_location.world_coordinates);
                auto size = volumetric_mask->get_volume()->GetLargestPossibleRegion().GetSize();
                if(size[0]> current_mouse_location.image_coordinates[0] && 
                    size[1]> current_mouse_location.image_coordinates[1] && 
                    size[2]> current_mouse_location.image_coordinates[2] && 
                    current_mouse_location.image_coordinates[0]>= 0 &&
                    current_mouse_location.image_coordinates[1]>= 0 &&
                    current_mouse_location.image_coordinates[2]>= 0)
                    current_mouse_location.value= volumetric_mask->get_volume()->GetPixel(current_mouse_location.image_coordinates);
                else 
                    current_mouse_location.value = -100.0;
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
                return false;
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
                if (current_path_state == DRAWPATH)
                    current_stroke.add_point(homogenenous_transformation, SkPoint::Make((float)xpos, (float)ypos));
                return true;
            }
    
            if(!is_options && interpreter.status() & ~curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED && 
                interpreter.check(curan::ui::InterpreterStatus::INSIDE_ALLOCATED_AREA | 
                                    curan::ui::InterpreterStatus::MOUSE_CLICKED_LEFT_EVENT) ){
                set_current_state(SliderStates::PRESSED);
                if (current_path_state == DRAWPATH)
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