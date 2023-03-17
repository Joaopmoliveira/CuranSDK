#include "userinterface/widgets/OpenIGTLinkViewer.h"

#include <variant>
#include "utils/Overloading.h"

namespace curan {
	namespace ui {
		OpenIGTLinkViewer::OpenIGTLinkViewer(Info& info) {
			text_font = info.text_font;

			std::string type = "Type";
			sk_sp<SkTextBlob> text_type = SkTextBlob::MakeFromString(type.c_str(), text_font);
			table_headers.push_back(text_type);

			std::string name = "Name";
			sk_sp<SkTextBlob> text_name = SkTextBlob::MakeFromString(name.c_str(), text_font);
			table_headers.push_back(text_name);

			std::string timestamp = "Timestamp (ms)";
			sk_sp<SkTextBlob> text_timestamp = SkTextBlob::MakeFromString(timestamp.c_str(), text_font);
			table_headers.push_back(text_timestamp);

			std::string freq = "Frequency (Hz)";
			sk_sp<SkTextBlob> text_freq = SkTextBlob::MakeFromString(freq.c_str(), text_font);
			table_headers.push_back(text_freq);

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
		}

		std::shared_ptr<OpenIGTLinkViewer> OpenIGTLinkViewer::make(Info& info) {
			return std::make_shared<OpenIGTLinkViewer>(info);
		}

		void OpenIGTLinkViewer::process_message(igtl::MessageBase::Pointer pointer) {
			
		}

		drawablefunction OpenIGTLinkViewer::impldraw() {
			auto callab = [this](SkCanvas* canvas) {
				std::lock_guard<std::mutex> g{ get_mutex() };
				paint.setColor(SK_ColorWHITE);
				paint.setStrokeWidth(1);
				SkAutoCanvasRestore restore(canvas, true);
				canvas->clipRect(widget_rect);
				canvas->clear(SK_ColorBLACK);
				center_debug_mode = { widget_rect.fRight - debug_mode_radius,widget_rect.fTop + debug_mode_radius };

				if (in_debug_mode) {

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

					int number_of_cells = (height / 2.0) / (double)(DK_DEFAULT_TEXT_SIZE + 5);

					SkRect renctangle = SkRect::MakeXYWH(x_init, y_init, width / 4, DK_DEFAULT_TEXT_SIZE + 5);

					for (int index = { 0 }; index < 4; ++index) {
						canvas->drawRect(renctangle, paint);
						SkRect text_blob_rect = table_headers[index]->bounds();
						canvas->drawTextBlob(table_headers[index], renctangle.x() + (renctangle.width() - text_blob_rect.width()) / 2, renctangle.y() + renctangle.height() - text_blob_rect.bottom(), paint_text);
						renctangle.offsetTo(renctangle.x() + renctangle.width(), renctangle.y());
					}

					renctangle.offsetTo(x_init, y_init + renctangle.height());

					std::map<std::string, MessageInfo>::iterator it = table_conversion.begin();
					std::map<std::string, MessageInfo>::iterator clicked_it = table_conversion.end();
					for (int index = 0; index < number_of_cells; ++index) {
						if (renctangle.fBottom > last_pressed_position.ypos && renctangle.fTop < last_pressed_position.ypos) {
							paint.setStyle(SkPaint::kFill_Style);
							paint.setColor(SK_ColorDKGRAY);
							clicked_it = it;
						}
						else {
							paint.setStyle(SkPaint::kStroke_Style);
							paint.setColor(SK_ColorWHITE);
						}

						if (it == table_conversion.end()) {
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
						else {
							MessageInfo& info = it->second;
							SkRect rect = renctangle;
							canvas->drawRect(rect, paint);
							std::string type = info.message_type;
							canvas->drawSimpleText(type.data(), type.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							std::string device_name = info.message_devide_name;
							canvas->drawSimpleText(device_name.data(), device_name.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(info.timestamp.time_since_epoch()).count());
							canvas->drawSimpleText(timestamp.data(), timestamp.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
							rect.offsetTo(rect.x() + rect.width(), rect.y());
							canvas->drawRect(rect, paint);
							std::string frequency = std::to_string(info.frequency);
							canvas->drawSimpleText(frequency.data(), frequency.size(), SkTextEncoding::kUTF8, rect.x(), rect.bottom(), text_font, paint_text);
							renctangle.offsetTo(renctangle.x(), renctangle.y() + renctangle.height());
							++it;
						}

					}
					std::string preview = "Preview: ";
					canvas->drawSimpleText(preview.data(), preview.size(), SkTextEncoding::kUTF8, x_init, y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5), text_font, paint_text);

					if (clicked_it != table_conversion.end()) {
						MessageInfo& selected_message = clicked_it->second;
						switch (selected_message.getType()) {
						case MessageType::IMAGE:
						{
							SkScalar y = y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5);
							SkSamplingOptions options;
							sk_sp<SkImage> surf;
							igtl::ImageMessage::Pointer img;
							if (selected_message.getImage(img, surf)) {
								canvas->drawImage(surf, widget_rect.x() + (widget_rect.width() - surf->width()) / 2.0, y);
							}
						}
						break;
						case MessageType::TRANSFORM:
						{
							SkScalar y = y_init + (number_of_cells + 2.0) * (DK_DEFAULT_TEXT_SIZE + 5);
							// Retrive the transform data
							igtl::TransformMessage::Pointer transf;
							if (selected_message.getTransform(transf)) {
								igtl::Matrix4x4 matrix;
								transf->GetMatrix(matrix);
								for (int i = 0; i < 4; ++i) {
									std::string row = "[";
									for (int j = 0; j < 4; ++j)
										row += curan::utils::to_string_with_precision(matrix[i][j], 2) + " , ";
									row += "]";
									SkRect bound_individual_name;
									text_font.measureText(row.c_str(), row.size(), SkTextEncoding::kUTF8, &bound_individual_name);
									canvas->drawSimpleText(row.c_str(), row.size(), SkTextEncoding::kUTF8, (widget_rect.width() - bound_individual_name.width()) / 2, y + bound_individual_name.height(), text_font, paint_text);
									y += bound_individual_name.height() + 5;
								}
							}
						}
						break;
						default:

							break;
						}
					}
				}
				else {
					canvas->drawCircle(center_debug_mode, debug_mode_radius, paint);
					canvas->drawTextBlob(debug_glyph, center_debug_mode.fX, center_debug_mode.fY - debug_glyph->bounds().centerY(), paint_text);
					std::map<std::string, MessageInfo>::iterator it = table_conversion.begin();
					while (it != table_conversion.end())
					{
						if (it->second.getType() == MessageType::IMAGE) {
							SkSamplingOptions options;
							sk_sp<SkImage> surf;
							igtl::ImageMessage::Pointer img;
							it->second.getImage(img, surf);
							canvas->drawImage(surf, widget_rect.x() + (widget_rect.width() - surf->width()) / 2.0, widget_rect.y() + (widget_rect.height() - surf->height()) / 2.0);
							return;
						}
						++it;
					}

				}
			};
			return callab;
		}

		callablefunction OpenIGTLinkViewer::implcall() {
			auto lamb = [this](Signal sig) {

				std::lock_guard<std::mutex> g{ get_mutex() };

				std::visit(utils::overloaded{
				[this](Empty arg) {

					},
				[this](Move arg) {

					},
				[this](Press arg) {;

					},
				[this](Scroll arg) {;

					},
				[this](Key arg) {

					},
				[this](Unpress arg) {;

					},
				[this](ItemDropped arg) {;

					} },
					sig);
			};
			return lamb;
		}

		bool OpenIGTLinkViewer::interacts(double x, double y) {
			return true;
		}
	}
}