#include "userinterface/widgets/TwoDimensionalViewer.h"
#include "utils/Overloading.h"

namespace curan{
namespace ui{

void ItkMask::container_resized(const SkMatrix &inverse_homogenenous_transformation)
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

std::optional<curan::ui::Stroke> ItkMask::draw(SkCanvas *canvas, const SkMatrix &inverse_homogenenous_transformation, const SkMatrix &homogenenous_transformation, const SkPoint &point, bool is_highlighting, SkPaint &paint_stroke, SkPaint &paint_square, const SkFont &text_font, bool is_pressed)
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

}
}