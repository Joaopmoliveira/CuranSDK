#ifndef CURAN_ICON_EXPLORER_HEADER_FILE_
#define CURAN_ICON_EXPLORER_HEADER_FILE_

#include "Drawable.h"
#include "utils/Cancelable.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"

namespace curan {
	namespace ui {
        class ItemExplorer : public  Drawable , public utilities::Lockable, public SignalProcessor<ItemExplorer> {
		public:
			struct Info {
				SkColor color_background_left;
				SkColor color_hover;
				SkColor color_selected;
				SkColor color_waiting;
				SkColor color_background_right;
				SkPaint text_paint;
				SkFont font;
			};

			struct Item {
				sk_sp<SkImage> image;
				SkRect current_pos;
				bool is_selected = false;
				sk_sp<SkTextBlob> text;
			};

            std::list<int> current_selected_identifiers;

			int selected_image_identifier = 0;
			SkRect preview_rectangle;
			bool is_exclusive = true;

			SkPaint paint_image_background;
			SkPaint paint_background;
			SkPaint text_paint;
			SkFont font;

			SkColor color_background;
			
			SkColor color_waiting;
			SkColor color_hover;
			SkColor color_selected;

			SkScalar vertical_scroll = 0.0;

			MouseMoveSignal current_mouse_position = { -1.0 , -1.0 };

			std::atomic<float> maximum_height{ 0 };
			std::atomic<float> current_height_offset{ 0 };

			std::map<int, Item> item_list;

			ItemExplorer(Info& info);
			void draw(SkCanvas* canvas, SkRect& widget_rect) override;
			static std::shared_ptr<ItemPreview> make(Info& info);
			void callback(Signal signal, bool* interacted) override;
			
			/*
			* The method returns the current index which is selected by the user
			*/
			int getselectedindex(std::list<int>& list_selected_indexes);
			
			/*
			* The method updates the current listing by adding a new item to the listing
			*/
			bool update(Item item_to_add,int identifier);

			/*
			* The remove method removes a listing from an external providir
			*/
			void remove(int identifier);

        }
    }
}