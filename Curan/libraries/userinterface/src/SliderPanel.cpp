#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

namespace curan {
namespace ui {

	void Mask::container_resized(const SkMatrix &inverse_homogenenous_transformation)
	{
		for (auto &stro : recorded_strokes)
			stro.second.container_resized(inverse_homogenenous_transformation);
	}

	void Mask::draw(SkCanvas *canvas,const SkMatrix& homogenenous_transformation,const SkPoint& point, bool is_highlighting,SkPaint& paint_stroke,SkPaint& paint_square,const SkFont& text_font)
	{
		if (is_highlighting)
		{
			double minimum = std::numeric_limits<double>::max();
			auto minimum_index = recorded_strokes.end();
			for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
			{
				double local = begin->second.distance(homogenenous_transformation, point);
				if (minimum > local)
				{
					minimum = local;
					minimum_index = begin;
				}

				if (begin->second.normalized_recorded_points.size() == 1)
					canvas->drawPoint(begin->second.begin_point, paint_stroke);
				else
					canvas->drawPath(begin->second.rendered_path, paint_stroke);
			}

			for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
			{
				if (begin->second.normalized_recorded_points.size() == 1)
				{
					auto point = begin->second.begin_point;
					paint_square.setColor(SkColorSetARGB(60, 0, 0, 0));
					canvas->drawCircle(SkPoint::Make(point.fX + 5, point.fY + 5), 10, paint_square);
					paint_square.setColor(SK_ColorGREEN);
					std::string indentifier = begin->second.identifier + std::to_string(begin->first);
					canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
				}
				else
				{
					auto point = begin->second.begin_point;
					paint_square.setColor(SkColorSetARGB(155, 0, 0, 0));
					canvas->drawCircle(SkPoint::Make(point.fX + 10, point.fY + 10), 20, paint_square);
					paint_square.setColor(SK_ColorGREEN);
					std::string indentifier = begin->second.identifier + std::to_string(begin->first);
					paint_stroke.setStrokeWidth(0.5f);
					canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
					paint_stroke.setStrokeWidth(8);
				}
			}

			if (minimum_index != recorded_strokes.end() && minimum < 0.02f)
			{
				paint_stroke.setStrokeWidth(14);
				paint_stroke.setColor(SkColorSetARGB(125, 0x00, 0xFF, 0x00));
				if (minimum_index->second.normalized_recorded_points.size() == 1)
					canvas->drawPoint(minimum_index->second.begin_point, paint_stroke);
				else
					canvas->drawPath(minimum_index->second.rendered_path, paint_stroke);
				paint_stroke.setStrokeWidth(8);
				paint_stroke.setColor(SK_ColorGREEN);
			}
		}
		else
		{
			for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
			{
				if (begin->second.normalized_recorded_points.size() == 1)
					canvas->drawPoint(begin->second.begin_point, paint_stroke);
				else
					canvas->drawPath(begin->second.rendered_path, paint_stroke);
			}

			for (auto begin = recorded_strokes.begin(); begin != recorded_strokes.end(); ++begin)
			{
				if (begin->second.normalized_recorded_points.size() == 1)
				{
					auto point = begin->second.begin_point;
					paint_square.setColor(SkColorSetARGB(60, 0, 0, 0));
					canvas->drawCircle(SkPoint::Make(point.fX + 5, point.fY + 5), 10, paint_square);
					paint_square.setColor(SK_ColorGREEN);
					std::string indentifier = begin->second.identifier + std::to_string(begin->first);
					canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
				}
				else
				{
					auto point = begin->second.begin_point;
					paint_square.setColor(SkColorSetARGB(155, 0, 0, 0));
					canvas->drawCircle(SkPoint::Make(point.fX + 10, point.fY + 10), 20, paint_square);
					paint_square.setColor(SK_ColorGREEN);
					std::string indentifier = begin->second.identifier + std::to_string(begin->first);
					paint_stroke.setStrokeWidth(0.5f);
					canvas->drawSimpleText(indentifier.data(), indentifier.size(), SkTextEncoding::kUTF8, point.fX + 10, point.fY + 10, text_font, paint_square);
					paint_stroke.setStrokeWidth(8);
				}
			}
        }
    }


	void SlidingPanel::query_if_required()
	{
		size_t previous = _current_index;
		_current_index = std::round(current_value * (masks.size() - 1));
		if (previous != _current_index)
		{
			background = extract_slice_from_volume(_current_index);
		}
		previous = _current_index;
	}

	Mask& SlidingPanel::SlidingPanel::current_mask()
	{
		_current_index = std::round(current_value * (masks.size() - 1));
		return masks[_current_index];
	}

	SlidingPanel::image_info SlidingPanel::extract_slice_from_volume(size_t index)
	{
		extract_filter = ExtractFilterType::New();
		extract_filter->SetDirectionCollapseToSubmatrix();
		extract_filter->SetInput(contained_volume);

		ImageType::RegionType inputRegion = contained_volume->GetBufferedRegion();
		ImageType::SpacingType spacing = contained_volume->GetSpacing();
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

		image_info info;

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

		return info;
	}

	SlidingPanel::SlidingPanel(curan::ui::IconResources &other, ImageType::Pointer in_contained_volume, Direction in_direction) : system_icons{other}
	{
		update_volume(in_contained_volume, in_direction);
		query_if_required();
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

		options = SkSamplingOptions();

		const char *fontFamily = nullptr;
		SkFontStyle fontStyle;
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
		auto typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

		text_font = SkFont(typeface, 20, 1.0f, 0.0f);
		text_font.setEdging(SkFont::Edging::kAntiAlias);
	}

	void SlidingPanel::insert_in_map(const curan::ui::PointCollection &future_stroke)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		auto &mask = current_mask();
		mask.try_emplace(counter, curan::ui::Stroke{future_stroke.normalized_recorded_points, inverse_homogenenous_transformation});
		++counter;
	}

	std::unique_ptr<SlidingPanel> SlidingPanel::make(curan::ui::IconResources &other, ImageType::Pointer in_contained_volume, Direction in_direction)
	{
		std::unique_ptr<SlidingPanel> button = std::unique_ptr<SlidingPanel>(new SlidingPanel{other, in_contained_volume, in_direction});
		return button;
	}

	SlidingPanel::~SlidingPanel()
	{
	}

	void SlidingPanel::compile()
	{
	}

	void SlidingPanel::update_volume(ImageType::Pointer in_contained_volume, Direction in_direction)
	{
		std::lock_guard<std::mutex> g{get_mutex()};
		contained_volume = in_contained_volume;
		direction = in_direction;
		ImageType::RegionType inputRegion = contained_volume->GetBufferedRegion();
		ImageType::SizeType size = inputRegion.GetSize();
		_current_index = std::floor(current_value * (size[direction] - 1));
		masks.clear();
		masks = std::vector<Mask>(size[direction]);
		dragable_percent_size = 1.0 / size[direction];
	}

	void SlidingPanel::framebuffer_resize(const SkRect &new_page_size)
	{
		auto pos = get_position();
		reserved_drawing_space = SkRect::MakeLTRB(pos.fLeft, pos.fTop, pos.fRight, pos.fBottom - size_of_slider_in_height);
		reserved_slider_space = SkRect::MakeLTRB(pos.fLeft, pos.fBottom - size_of_slider_in_height, pos.fRight, pos.fBottom);
		double width = 1;
		double height = 1;

		size_t address = (size_t) this;

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
		homogenenous_transformation.invert(&inverse_homogenenous_transformation);

		std::lock_guard<std::mutex> g{get_mutex()};
		current_mask().container_resized(inverse_homogenenous_transformation);
		set_size(pos);
		return;
	}


	curan::ui::drawablefunction SlidingPanel::draw()
	{
		auto lamb = [this](SkCanvas *canvas)
		{
			auto widget_rect = get_position();
			SkAutoCanvasRestore restore{canvas, true};
			canvas->drawRect(widget_rect, background_paint);

			if (background.image)
			{
				auto val = *background.image;
				auto image_display_surface = val.image;
				SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{1.0f / 3.0f, 1.0f / 3.0f});
				canvas->drawImageRect(image_display_surface, background_rect, opt);
			}

			canvas->drawPoints(SkCanvas::PointMode::kPoints_PointMode, current_stroke.transformed_recorded_points.size(), current_stroke.transformed_recorded_points.data(), paint_stroke);

			{
				std::lock_guard<std::mutex> g{get_mutex()};
				current_mask().draw(canvas,homogenenous_transformation,zoom_in.get_coordinates(),is_highlighting,paint_stroke,paint_square,text_font);
			}

			if (zoom_in)
				zoom_in.draw(canvas);

			slider_paint.setColor(slider_color);
			slider_paint.setStyle(SkPaint::kStroke_Style);

			SkRect dragable = SkRect::MakeXYWH(reserved_slider_space.x() + (reserved_slider_space.width() * (1 - dragable_percent_size)) * current_value, reserved_slider_space.y(), reserved_slider_space.width() * dragable_percent_size, reserved_slider_space.height());
			SkRect contained_squares = SkRect::MakeXYWH(reserved_slider_space.x(), reserved_slider_space.y(), reserved_slider_space.width() * dragable_percent_size, reserved_slider_space.height());
			canvas->drawRoundRect(reserved_slider_space, reserved_slider_space.height() / 2.0f, reserved_slider_space.height() / 2.0f, slider_paint);
			size_t increment_mask = 0;
			for (const auto &mask : masks)
			{
				slider_paint.setColor((increment_mask == _current_index) ? SK_ColorGREEN : hover_color);
				canvas->drawRoundRect(contained_squares, contained_squares.height() / 2.0f, contained_squares.height() / 2.0f, slider_paint);
				contained_squares.offset(dragable.width(), 0);
				++increment_mask;
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
			}

			canvas->drawRoundRect(dragable, reserved_slider_space.height() / 2.0f, reserved_slider_space.height() / 2.0f, slider_paint);
		};
		return lamb;
	}

	curan::ui::callablefunction SlidingPanel::call()
	{
		auto lamb = [this](curan::ui::Signal sig, curan::ui::ConfigDraw *config)
		{
			bool interacted = false;
			std::visit(curan::utilities::overloaded{[](curan::ui::Empty arg) {

													},
													[&](curan::ui::Move arg)
													{
														static curan::ui::Move previous_arg = arg;
														auto previous_state = get_current_state();
														auto current_state_local = get_current_state();
														zoom_in.store_position(SkPoint::Make((float)arg.xpos, (float)arg.ypos), get_size());
														if (reserved_drawing_space.contains(arg.xpos, arg.ypos) && current_state_local != SliderStates::PRESSED)
														{
															if (!is_highlighting)
															{
																if (is_pressed)
																{
																	current_stroke.add_point(homogenenous_transformation, SkPoint::Make((float)arg.xpos, (float)arg.ypos));
																}
																else if (!current_stroke.empty())
																{
																	insert_in_map(current_stroke);
																	current_stroke.clear();
																}
																interacted = true;
															}
															
														}
														else if (get_position().contains(arg.xpos, arg.ypos) && current_state_local == SliderStates::PRESSED)
														{
															auto offset_x = ((float)arg.xpos - reserved_slider_space.x()) / reserved_slider_space.width();
															auto current_val = get_current_value();
															current_val += offset_x - read_trigger();
															trigger(offset_x);
															set_current_value(current_val);
														}
														else
															current_state_local = SliderStates::WAITING;

														if (previous_state != current_state_local)
															interacted = true;
														previous_arg = arg;
														set_current_state(current_state_local);
													},
													[&](curan::ui::Press arg)
													{
														auto previous_state = get_current_state();
														auto current_state_local = get_current_state();
														if (reserved_drawing_space.contains(arg.xpos, arg.ypos))
														{
															if (current_stroke.normalized_recorded_points.size() == 1)
															{
																insert_in_map(current_stroke);
																current_stroke.clear();
															}
															current_stroke.add_point(homogenenous_transformation, SkPoint::Make((float)arg.xpos, (float)arg.ypos));
															is_pressed = true;
															interacted = true;
														}
														else if (reserved_slider_space.contains(arg.xpos, arg.ypos))
														{
															trigger((arg.xpos - reserved_slider_space.x()) / reserved_slider_space.width());
															current_state_local = SliderStates::PRESSED;
														}
														else
															current_state_local = SliderStates::WAITING;
														if (previous_state != current_state_local)
															interacted = true;
														set_current_state(current_state_local);
													},
													[&](curan::ui::Scroll arg)
													{
														auto previous_state = get_current_state();
														auto current_state_local = get_current_state();
														if (get_position().contains(arg.xpos, arg.ypos))
														{
															auto offsetx = (float)arg.xoffset / reserved_slider_space.width();
															auto offsety = (float)arg.yoffset / reserved_slider_space.width();
															auto current_val = get_current_value();
															current_val += (std::abs(offsetx) > std::abs(offsety)) ? offsetx : offsety;
															set_current_value(current_val);
															current_state_local = SliderStates::PRESSED;
														}
														else
														{
															current_state_local = SliderStates::WAITING;
														}
														if (previous_state != current_state_local)
															interacted = true;
														set_current_state(current_state_local);
													},
													[&](curan::ui::Unpress arg)
													{
														is_pressed = false;
														if (!current_stroke.empty())
														{
															insert_in_map(current_stroke);
															current_stroke.clear();
														}
														interacted = true;
														auto previous_state = get_current_state();
														auto current_state_local = get_current_state();
														if (reserved_slider_space.contains(arg.xpos, arg.ypos))
															current_state_local = SliderStates::HOVER;
														else
															current_state_local = SliderStates::WAITING;
														if (previous_state != current_state_local)
															interacted = true;
														set_current_state(current_state_local);
													},
													[&](curan::ui::Key arg)
													{
														if (arg.key == GLFW_KEY_A && arg.action == GLFW_PRESS)
														{
															if (zoom_in)
																zoom_in.deactivate();
															else
																zoom_in.activate();
															return;
														}

														if (arg.key == GLFW_KEY_S && arg.action == GLFW_PRESS)
														{
															is_highlighting = !is_highlighting;
															if (!current_stroke.empty())
															{
																insert_in_map(current_stroke);
																current_stroke.clear();
															}
														}
													},
													[](curan::ui::ItemDropped arg) {

													}},
					   sig);
			return interacted;
		};
		return lamb;
	};

}
}