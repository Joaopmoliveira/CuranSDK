#include "userinterface/widgets/DicomDisplay.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"
#include "geometry/Intersection.h"

namespace curan{
namespace ui{
    size_t VolumetricMask::counter = 0;
    
    VolumetricMask::VolumetricMask(ImageType::Pointer volume) : image{volume}
    {
        update_volume(volume);
    }
    
    void DicomMask::container_resized(const SkMatrix &inverse_homogenenous_transformation)
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
    
    std::optional<curan::ui::Stroke> DicomMask::draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed)
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
    
    void DicomViewer::query_if_required(bool force_update)
    {
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
    
    DicomViewer::image_info DicomViewer::extract_slice_from_volume(size_t index)
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
    
    DicomViewer::DicomViewer(curan::ui::IconResources &other, 
                            VolumetricMask *volume_mask, 
                            Direction in_direction) : volumetric_mask{volume_mask},
                                                        chached_pointer{volume_mask->get_volume().GetPointer()},
                                                        cached_number_of_geometries{volume_mask->geometries().size()}, 
                                                        system_icons{other}
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
    
    void DicomViewer::insert_in_map(const curan::ui::PointCollection &future_stroke)
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
    
    std::unique_ptr<DicomViewer> DicomViewer::make(curan::ui::IconResources &other, VolumetricMask *volume_mask, Direction in_direction)
    {
        std::unique_ptr<DicomViewer> button = std::unique_ptr<DicomViewer>(new DicomViewer{other, volume_mask, in_direction});
        return button;
    }
    
    DicomViewer::~DicomViewer()
    {
    }
    
    void DicomViewer::compile()
    {
        if(compiled){
            throw std::runtime_error("cannot compile twice");
        }
        //TODO: 
        compiled = true;
    }
    
    void DicomViewer::update_volume(VolumetricMask *volume_mask, Direction in_direction)
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
    
    void DicomViewer::internal_framebuffer_recomputation(){
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
                y_location += max_height+buffer_sideways;
                x_location = reserved_drawing_space.fLeft+buffer_sideways;
            }
        }  
        
        return;

    }
    
    void DicomViewer::framebuffer_resize(const SkRect &new_page_size)
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
                y_location += max_height+buffer_sideways;
                x_location = reserved_drawing_space.fLeft+buffer_sideways;
            }
        }  
        
        return;
    }
    
    curan::ui::drawablefunction DicomViewer::draw()
    {
        auto lamb = [this](SkCanvas *canvas)
        {
        if(!compiled){
            throw std::runtime_error("must compile the DicomViewer before drawing operations");
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
    
            if(is_panel_selected){
                std::string pixel_value;
                if(current_mouse_location.value >= 0.0){
                    pixel_value = "Pixel value: " + std::to_string(current_mouse_location.value);
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
    
    curan::ui::callablefunction DicomViewer::call()
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
                query_if_required(true);
                framebuffer_resize(SkRect::MakeWH(0,0));
            }
    
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
    
    }
    }
    