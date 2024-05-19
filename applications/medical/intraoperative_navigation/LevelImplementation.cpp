#include "LevelImplementation.h"

    void Level::draw(SkCanvas *canvas, int bubble_position, LevelOrientation orientation)
    {
        auto surface_height = canvas->getSurface()->height();
        auto surface_width = canvas->getSurface()->width();
        SkPaint greenPaint;
        greenPaint.setAntiAlias(true);
        greenPaint.setColor(SK_ColorGREEN);
        switch (orientation)
        {
        case LevelOrientation::horizontal:
        {
            SkPaint level_horizontal;
            SkPoint points_level[2] = {SkPoint::Make(0.0, 0.0),
                                       SkPoint::Make(0.0, 60.0)};
            SkColor colors_level[2] = {SkColorSetARGB(255.0, 1.0, 65.0, 0.0),
                                       SkColorSetARGB(255.0, 114.0, 213.0, 0.0)};
            level_horizontal.setShader(SkGradientShader::MakeLinear(points_level, colors_level, NULL, 2, SkTileMode::kClamp, 0, NULL));
            SkPath path_level;
            path_level.moveTo(60, 0);             // coner 1
            path_level.lineTo(surface_width-60, 0);  // coner 2
            path_level.lineTo(surface_width-60, 60); // coner 3
            path_level.lineTo(60, 60);            // coner 4
            canvas->drawPath(path_level, level_horizontal);

            SkPaint level_bubble;
            level_bubble.setAntiAlias(true);
            SkColor colors_buble[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                       SkColorSetARGB(255.0, 180.0, 200.0, 0.0)};
            SkColor colors[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                 SkColorSetARGB(255.0, 250.0, 250.0, 250.0)};

            level_bubble.setShader(SkShaders::Blend(
                SkBlendMode::kSoftLight, SkGradientShader::MakeTwoPointConical(SkPoint::Make(bubble_position, 30.0), 30.0f, SkPoint::Make(bubble_position, 6.0), 2.0f, colors, nullptr, 2, SkTileMode::kClamp, 0, nullptr), SkGradientShader::MakeRadial(SkPoint::Make(bubble_position, 30.0), 15.0, colors_buble, NULL, 2, SkTileMode::kClamp)));

            canvas->drawOval(SkRect::MakeXYWH(bubble_position - 30, 10.0, 50, 40), level_bubble);
            SkPaint black_lines;
            black_lines.setAntiAlias(true);
            black_lines.setColor(SK_ColorBLACK);
            black_lines.setStroke(true);
            black_lines.setStrokeWidth(3);
            canvas->drawLine(SkPoint::Make(surface_width / 2.0 - 90, 0.0), SkPoint::Make(surface_width / 2.0 - 90, 60.0), black_lines);
            canvas->drawLine(SkPoint::Make(surface_width / 2.0 + 90, 0.0), SkPoint::Make(surface_width / 2.0 + 90, 60.0), black_lines);
        }
        break;
        case LevelOrientation::vertical:
        {
            SkPaint level_vertical;
            SkPoint points_level[2] = {SkPoint::Make(0.0, 0.0),
                                       SkPoint::Make(60.0, 0.0)};
            SkColor colors_level[2] = {SkColorSetARGB(255.0, 1.0, 65.0, 0.0),
                                       SkColorSetARGB(255.0, 114.0, 213.0, 0.0)};
            level_vertical.setShader(SkGradientShader::MakeLinear(points_level, colors_level, NULL, 2, SkTileMode::kClamp, 0, NULL));
            SkPath path_level;
            path_level.moveTo(0, 60);             // coner 1
            path_level.lineTo(60, 60);  // coner 2
            path_level.lineTo(60, surface_height-60); // coner 3
            path_level.lineTo(0, surface_height-60);            // coner 4
            canvas->drawPath(path_level, level_vertical);

            SkPaint level_bubble;
            level_bubble.setAntiAlias(true);
            SkColor colors_buble[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                       SkColorSetARGB(255.0, 180.0, 200.0, 0.0)};
            SkColor colors[2] = {SkColorSetARGB(255.0, 148.0, 245.0, 0.0),
                                 SkColorSetARGB(255.0, 250.0, 250.0, 250.0)};

            level_bubble.setShader(SkShaders::Blend(
                SkBlendMode::kSoftLight, SkGradientShader::MakeTwoPointConical(SkPoint::Make(30.0,bubble_position), 30.0f, SkPoint::Make(6.0,bubble_position), 2.0f, colors, nullptr, 2, SkTileMode::kClamp, 0, nullptr), SkGradientShader::MakeRadial(SkPoint::Make(30.0,bubble_position), 15.0, colors_buble, NULL, 2, SkTileMode::kClamp)));

            canvas->drawOval(SkRect::MakeXYWH(10.0,bubble_position - 30, 40, 50), level_bubble);
            SkPaint black_lines;
            black_lines.setAntiAlias(true);
            black_lines.setColor(SK_ColorBLACK);
            black_lines.setStroke(true);
            black_lines.setStrokeWidth(3);
            canvas->drawLine(SkPoint::Make(0.0,surface_height / 2.0 - 90), SkPoint::Make(60.0,surface_height / 2.0 - 90), black_lines);
            canvas->drawLine(SkPoint::Make(0.0,surface_height / 2.0 + 90), SkPoint::Make(60.0,surface_height / 2.0 + 90), black_lines);
        }
            break;
        }
    }