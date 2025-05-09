#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include <variant>
#include "utils/Overloading.h"
#include "utils/StringManipulation.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "userinterface/widgets/definitions/Interactive.h"

namespace curan
{
	namespace ui
	{

		void create_skia_image(ImageMessage &message)
		{
			int width, height, depth = 0;
			message.image->GetDimensions(width, height, depth);

			if (message.image->GetNumComponents() != 1)
				return;

			switch (message.image->GetScalarType())
			{
			case igtl::ImageMessage::TYPE_UINT16:
			case igtl::ImageMessage::TYPE_UINT32:
			case igtl::ImageMessage::TYPE_FLOAT32:
			case igtl::ImageMessage::TYPE_FLOAT64:
			case igtl::ImageMessage::TYPE_INT16:
			case igtl::ImageMessage::TYPE_INT32:
				return;
			case igtl::ImageMessage::TYPE_INT8:
				message.information = SkImageInfo::Make(width, height, SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType, SkColorSpace::MakeSRGBLinear());
				break;
			case igtl::ImageMessage::TYPE_UINT8:
				message.information = SkImageInfo::Make(width, height, SkColorType::kGray_8_SkColorType, SkAlphaType::kPremul_SkAlphaType, SkColorSpace::MakeSRGBLinear());
				break;
			default:
				return;
			}

			auto pix = SkPixmap(message.information, message.image->GetScalarPointer(), message.information.bytesPerPixel() * message.information.width());
			message.skia_image = SkSurfaces::WrapPixels(pix)->makeImageSnapshot();
		};

		void set_skia_image(ImageMessage &message)
		{
			if (message.image->GetImageSize() == message.information.bytesPerPixel() * message.information.width() * message.information.height())
			{
				auto pix = SkPixmap(message.information, message.image->GetScalarPointer(), message.information.bytesPerPixel() * message.information.width());
				message.skia_image = SkSurfaces::WrapPixels(pix)->makeImageSnapshot();
			}
		};

		OpenIGTLinkViewer::OpenIGTLinkViewer() : last_pressed_position{Press::LEFT,-20000.0, -20000.0}
		{
			text_font = SkFont(defaultTypeface(), 10, 1.0f, 0.0f);
			text_font.setEdging(SkFont::Edging::kAntiAlias);
			set_size(SkRect::MakeWH(0.001, 0.001));
		}

		std::unique_ptr<OpenIGTLinkViewer> OpenIGTLinkViewer::make()
		{
			std::unique_ptr<OpenIGTLinkViewer> igtlviewer = std::unique_ptr<OpenIGTLinkViewer>(new OpenIGTLinkViewer());
			return igtlviewer;
		}

		void OpenIGTLinkViewer::process_message(igtl::MessageBase::Pointer pointer)
		{
			std::lock_guard<std::mutex> g{get_mutex()};
			bool message_previously_received = false;
			for (auto &val : container.received_messages)
			{
				std::visit(utilities::overloaded{[pointer, &message_previously_received](TransformMessage &message)
												 {
													 std::string ident = pointer->GetDeviceName() + pointer->GetMessageType();
													 if (!ident.compare(message.identifier))
													 {
														 message_previously_received = true;
														 igtl::TransformMessage::Pointer message_body = igtl::TransformMessage::New();
														 if (message_body->Copy(pointer) && message_body->Unpack(1) == igtl::MessageBase::UNPACK_BODY)
														 {
															 auto current_time_point = std::chrono::high_resolution_clock::now();
															 message.frequency = 1.0 / std::chrono::duration<double>(current_time_point - message.received_instant).count();
															 message.received_instant = current_time_point;
															 message.transform = message_body;
														 }
													 }
												 },
												 [pointer, &message_previously_received](ImageMessage &message)
												 {
													 std::string ident = pointer->GetDeviceName() + pointer->GetMessageType();
													 if (!ident.compare(message.identifier))
													 {
														 message_previously_received = true;
														 igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
														 if (message_body->Copy(pointer) && message_body->Unpack(1) == igtl::MessageBase::UNPACK_BODY)
														 {
															 auto current_time_point = std::chrono::high_resolution_clock::now();
															 message.frequency = 1.0 / std::chrono::duration<double>(current_time_point - message.received_instant).count();
															 message.received_instant = current_time_point;
															 message.image = message_body;
															 set_skia_image(message);
														 }
													 }
												 }},
						   val);
			}
			if (!message_previously_received)
			{
				if (!pointer->GetMessageType().compare("TRANSFORM"))
				{
					TransformMessage message;
					message.frequency = 0.0;
					message.identifier = pointer->GetDeviceName() + pointer->GetMessageType();
					message.received_instant = std::chrono::high_resolution_clock::now();
					igtl::TransformMessage::Pointer message_body = igtl::TransformMessage::New();
					message_body->Copy(pointer);
					int c = message_body->Unpack(1);
					if (c & igtl::MessageHeader::UNPACK_BODY)
					{
						message.transform = message_body;
						container.received_messages.push_back(message);
					}
					return;
				}
				if (!pointer->GetMessageType().compare("IMAGE"))
				{
					ImageMessage message;
					message.frequency = 0.0;
					message.identifier = pointer->GetDeviceName() + pointer->GetMessageType();
					message.received_instant = std::chrono::high_resolution_clock::now();
					igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
					message_body->Copy(pointer);
					int c = message_body->Unpack(1);
					if (c & igtl::MessageHeader::UNPACK_BODY)
					{
						message.image = message_body;
						create_skia_image(message);
						container.received_messages.push_back(message);
					}
					return;
				}
			}
		}

		drawablefunction OpenIGTLinkViewer::draw()
		{
			if (!compiled)
				throw std::runtime_error("must compile the button before drawing operations");
			auto callab = [this](SkCanvas *canvas)
			{
				paint.setColor(SK_ColorWHITE);
				paint.setStrokeWidth(1);
				SkAutoCanvasRestore restore(canvas, true);
				SkRect widget_rect = get_position();

				auto container = get_container();
				canvas->clipRect(widget_rect);
				canvas->clear(SK_ColorBLACK);
				center_debug_mode = {widget_rect.fRight - debug_mode_radius * 2, widget_rect.fTop + debug_mode_radius};
				if (!in_debug_mode)
				{
					canvas->drawCircle(center_debug_mode, debug_mode_radius, paint);
					canvas->drawTextBlob(debug_glyph, center_debug_mode.fX, center_debug_mode.fY - debug_glyph->bounds().centerY(), paint_text);
					for (auto &mess : container.received_messages)
					{
						std::visit(utilities::overloaded{[](TransformMessage message) {

														 },
														 [canvas, widget_rect, this](ImageMessage message)
														 {
															 SkSamplingOptions opt = SkSamplingOptions(SkCubicResampler{1.0 / 3, 1.0 / 3});
															 SkRect current_selected_image_rectangle = compute_bounded_rectangle(widget_rect, message.skia_image->width(), message.skia_image->height());
															 if (paint_compliant_filtered_image)
																 canvas->drawImageRect(message.skia_image, current_selected_image_rectangle, opt, &(*paint_compliant_filtered_image));
															 else
																 canvas->drawImageRect(message.skia_image, current_selected_image_rectangle, opt);
														 }},
								   mess);
					}
				}
				else
				{
					paint.setColor(SK_ColorGRAY);
					paint.setStyle(SkPaint::kFill_Style);
					canvas->drawCircle(center_debug_mode, debug_mode_radius, paint);
					paint.setColor(SK_ColorWHITE);
					paint.setStyle(SkPaint::kStroke_Style);
					canvas->drawTextBlob(debug_glyph, center_debug_mode.fX, center_debug_mode.fY - debug_glyph->bounds().centerY(), paint_text);

					SkScalar height = widget_rect.height() * 0.95;
					SkScalar width = widget_rect.width() * 0.95;
					double x_init = widget_rect.x() + (widget_rect.width() - width) / 2;
					double y_init = widget_rect.y() + (widget_rect.height() - height) / 2;

					int number_of_cells = std::floor((height / 2.0) / (DEFAULT_TEXT_SIZE + 5.0));
					SkRect renctangle = SkRect::MakeXYWH(x_init, y_init, width / 4, DEFAULT_TEXT_SIZE + 5);
					auto table_headers = get_table_header();

					for (const auto &val : table_headers)
					{
						canvas->drawRect(renctangle, paint);
						SkRect text_blob_rect = val->bounds();
						canvas->drawTextBlob(val, renctangle.x() + (renctangle.width() - text_blob_rect.width()) / 2, renctangle.y() + renctangle.height() - text_blob_rect.bottom(), paint_text);
						renctangle.offsetTo(renctangle.x() + renctangle.width(), renctangle.y());
					}

					renctangle.offsetTo(x_init, y_init + renctangle.height());

					auto it = container.received_messages.begin();
					auto clicked = container.received_messages.end();

					for (int index = 0; index < number_of_cells; ++index)
					{
						if (renctangle.fBottom > last_pressed_position.ypos && renctangle.fTop < last_pressed_position.ypos)
						{
							paint.setStyle(SkPaint::kFill_Style);
							paint.setColor(SK_ColorDKGRAY);
							clicked = it;
						}
						else
						{
							paint.setStyle(SkPaint::kStroke_Style);
							paint.setColor(SK_ColorWHITE);
						}

						if (it == container.received_messages.end())
						{
							SkRect rect = renctangle;
							canvas->drawRect(rect, paint);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
						}
						else
						{
							Message &val = *it;
							std::visit(utilities::overloaded{[&renctangle, canvas, this](TransformMessage message)
															 {
																 SkRect rect = renctangle;
																 canvas->drawRect(rect, paint);
																 std::string type = message.transform->GetMessageType();
																 canvas->drawSimpleText(type.data(), type.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string device_name = message.transform->GetDeviceName();
																 canvas->drawSimpleText(device_name.data(), device_name.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(message.received_instant.time_since_epoch()).count());
																 canvas->drawSimpleText(timestamp.data(), timestamp.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string frequency = std::to_string(message.frequency);
																 canvas->drawSimpleText(frequency.data(), frequency.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
															 },
															 [&renctangle, canvas, this](ImageMessage message)
															 {
																 SkRect rect = renctangle;
																 canvas->drawRect(rect, paint);
																 std::string type = message.image->GetMessageType();
																 canvas->drawSimpleText(type.data(), type.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string device_name = message.image->GetDeviceName();
																 canvas->drawSimpleText(device_name.data(), device_name.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(message.received_instant.time_since_epoch()).count());
																 canvas->drawSimpleText(timestamp.data(), timestamp.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 rect.offsetTo(rect.x() + rect.width(), rect.y());
																 canvas->drawRect(rect, paint);
																 std::string frequency = std::to_string(message.frequency);
																 canvas->drawSimpleText(frequency.data(), frequency.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
																 renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
															 }},
									   val);
							++it;
						}
					}

					std::string preview = "Preview: ";
					canvas->drawSimpleText(preview.data(), preview.size(), SkTextEncoding::kUTF8, x_init, y_init + (number_of_cells + 2.0) * (DEFAULT_TEXT_SIZE + 5), text_font, paint_text);
					y_init += (number_of_cells + 2.0) * (DEFAULT_TEXT_SIZE + 5);
					if (clicked != container.received_messages.end())
					{
						std::visit(utilities::overloaded{[&clicked, canvas, this, number_of_cells, y_init, x_init, widget_rect](TransformMessage &message)
														 {
															 SkScalar y = y_init;
															 // Retrive the transform data
															 igtl::Matrix4x4 matrix;
															 message.transform->GetMatrix(matrix);
															 for (int i = 0; i < 4; ++i)
															 {
																 std::string row = "[";
																 for (int j = 0; j < 4; ++j)
																	 row += utilities::to_string_with_precision(matrix[i][j], 2) + " , ";
																 row += "]";
																 SkRect bound_individual_name;
																 text_font.measureText(row.c_str(), row.size(), SkTextEncoding::kUTF8, &bound_individual_name);
																 canvas->drawSimpleText(row.c_str(), row.size(), SkTextEncoding::kUTF8, (widget_rect.width() - bound_individual_name.width()) / 2 + widget_rect.x(), y + bound_individual_name.height(), text_font, paint_text);
																 y += bound_individual_name.height() + 5;
															 }
														 },
														 [&clicked, canvas, this, number_of_cells, y_init, widget_rect](ImageMessage &message)
														 {
															 float image_width = message.skia_image->width();
															 float image_height = message.skia_image->height();
															 float current_selected_width = widget_rect.width();
															 float current_selected_height = widget_rect.height() - y_init;

															 auto allowed_area = SkRect::MakeXYWH(widget_rect.x(), y_init, current_selected_width, current_selected_height);
															 SkRect current_selected_image_rectangle = compute_bounded_rectangle(allowed_area, image_width, image_height);

															 SkSamplingOptions options;
															 if (paint_compliant_filtered_image)
																 canvas->drawImageRect(message.skia_image, current_selected_image_rectangle, options, &(*paint_compliant_filtered_image));
															 else
																 canvas->drawImageRect(message.skia_image, current_selected_image_rectangle, options);
														 }},
								   *clicked);
					}
				}
			};
			return callab;
		}

		callablefunction OpenIGTLinkViewer::call()
		{
			if (!compiled)
				throw std::runtime_error("must compile the button before drawing operations");
			auto lamb = [this](Signal sig, ConfigDraw *config)
			{
				auto check_inside_fixed_area = [this](double x, double y)
				{
					auto widget_rect = get_position();
					return widget_rect.contains(x, y);
				};
				interpreter.process(check_inside_fixed_area, check_inside_fixed_area, sig);

				if (interpreter.check(INSIDE_FIXED_AREA | MOUSE_CLICKED_LEFT_EVENT))
				{
					last_pressed_position = std::get<curan::ui::Press>(sig);
					float x = last_pressed_position.xpos - center_debug_mode.fX;
					float y = last_pressed_position.ypos - center_debug_mode.fY;
					if (x * x + y * y < debug_mode_radius * debug_mode_radius)
						in_debug_mode = !in_debug_mode;

					return true;
				}
				return false;
			};
			return lamb;
		}

		void OpenIGTLinkViewer::compile()
		{
			std::string type = "Type";
			sk_sp<SkTextBlob> text_type = SkTextBlob::MakeFromString(type.c_str(), text_font);
			table_headers[0] = text_type;

			std::string name = "Name";
			sk_sp<SkTextBlob> text_name = SkTextBlob::MakeFromString(name.c_str(), text_font);
			table_headers[1] = text_name;

			std::string timestamp = "Timestamp (ms)";
			sk_sp<SkTextBlob> text_timestamp = SkTextBlob::MakeFromString(timestamp.c_str(), text_font);
			table_headers[2] = text_timestamp;

			std::string freq = "Frequency (Hz)";
			sk_sp<SkTextBlob> text_freq = SkTextBlob::MakeFromString(freq.c_str(), text_font);
			table_headers[3] = text_freq;

			paint.setStyle(SkPaint::kStroke_Style);
			paint.setAntiAlias(true);
			paint.setStrokeWidth(2);
			paint.setColor(SK_ColorWHITE);

			paint_text.setStyle(SkPaint::kFill_Style);
			paint_text.setAntiAlias(true);
			paint_text.setStrokeWidth(1);
			paint_text.setColor(SK_ColorWHITE);

			std::string information = "i";
			debug_glyph = SkTextBlob::MakeFromString(information.c_str(), text_font);
			compiled = true;
		}

	}
}