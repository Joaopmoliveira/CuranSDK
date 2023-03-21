#ifndef CURAN_OPENIGTLINKVIEWER_HEADER_FILE_
#define CURAN_OPENIGTLINKVIEWER_HEADER_FILE_

#include "definitions/UIdefinitions.h"
#include <string>
#include <chrono>
#include <map>
#include <variant>
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "Drawable.h"
#include "Lockable.h"
#include "utils/Cancelable.h"

namespace curan {
	namespace ui {

		struct ImageMessage {
			std::chrono::time_point<std::chrono::high_resolution_clock> received_instant;
			std::string identifier;
			double frequency = 0.0;
			igtl::ImageMessage::Pointer image;
			sk_sp<SkImage> skia_image;
			SkPixmap pixel_array_nocopy;
			SkImageInfo information;
		};

		void create_skia_image(ImageMessage& message);

		void set_skia_image(ImageMessage& message);


		struct TransformMessage {
			std::chrono::time_point<std::chrono::high_resolution_clock> received_instant;
			std::string identifier;
			double frequency = 0.0;
			igtl::TransformMessage::Pointer transform;
		};

		using Message = std::variant<TransformMessage, ImageMessage>;

		struct MessageContainer {
			std::list<Message> received_messages;
		};

		class OpenIGTLinkViewer : public Drawable, Lockable<OpenIGTLinkViewer>, utils::Connectable<OpenIGTLinkViewer> {
			MessageContainer container;
			Press last_pressed_position;
			SkRect widget_rect;
			std::array<sk_sp<SkTextBlob>, 4> table_headers;

		public:
			struct Info {
				SkFont text_font;
				SkRect size;
			};

			OpenIGTLinkViewer(Info& info);
			static std::shared_ptr<OpenIGTLinkViewer> make(Info& info);
			void process_message(igtl::MessageBase::Pointer pointer);
			drawablefunction draw() override;
			callablefunction call() override;
			void framebuffer_resize() override;

			inline std::array<sk_sp<SkTextBlob>, 4>& get_table_header() {
				return table_headers;
			}

			inline MessageContainer& get_container() {
				return container;
			}

		private:

			SkFont text_font;
			SkPaint paint;
			SkPaint paint_text;
			bool in_debug_mode = false;
			SkPoint center_debug_mode = { 0.0f,0.0f };
			float debug_mode_radius = 10.0f;
			sk_sp<SkTextBlob> debug_glyph;
		};
	}
}

#endif