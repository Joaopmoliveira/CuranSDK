#ifndef CURAN_IMAGE_DISPLAY_HEADER_FILE_
#define CURAN_IMAGE_DISPLAY_HEADER_FILE_

namespace curan {
	namespace ui {

		template<bool is_static>
		class ImageDisplay {
		public:
			struct Info {
				SkFont text_font;
			};

			ImageDisplay(Info& info);
			static std::shared_ptr<ImageDisplay<is_static>> make(Info& info);
			void update_image(SkPixelmap pixelmap);

		};
	}
}

#endif