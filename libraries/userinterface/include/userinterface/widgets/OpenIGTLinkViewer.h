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

		class VoidMessage {

		};

		class OpenIGTLinkViewer : Drawable<OpenIGTLinkViewer>, Lockable<OpenIGTLinkViewer>, utils::Connectable<OpenIGTLinkViewer> {
		public:
			struct Info {
				SkFont text_font;
			};

			OpenIGTLinkViewer(Info& info);
			static std::shared_ptr<OpenIGTLinkViewer> make(Info& info);
			void process_message(igtl::MessageBase::Pointer pointer);
			drawablefunction impldraw();
			callablefunction implcall();
			bool interacts(double x, double y);

		private:
			SkFont text_font;
			SkPaint paint;
			SkPaint paint_text;
			std::vector<sk_sp<SkTextBlob>> table_headers;

			bool in_debug_mode = false;
			SkPoint center_debug_mode = { 0.0f,0.0f };
			float debug_mode_radius = 10.0f;
			sk_sp<SkTextBlob> debug_glyph;
			std::variant<igtl::TransformMessage::Pointer, igtl::ImageMessage::Pointer> message;

		};
	}
}

#endif