#ifndef CURAN_SIGNAL_PROCESSOR_HEADER_FILE_
#define CURAN_SIGNAL_PROCESSOR_HEADER_FILE_

#include "Signal.h"
#include <functional>

namespace curan
{
	namespace ui
	{
		struct ConfigDraw;

		template <typename S>
		struct SignalProcessor
		{
			using move_event = std::function<void(S*, Move, ConfigDraw* )>;
			using press_event = std::function<void(S *, Press, ConfigDraw* )>;
			using scroll_event = std::function<void(S *, Scroll, ConfigDraw* )>;
			using unpress_event = std::function<void(S *, Unpress, ConfigDraw* )>;
			using key_event = std::function<void(S *, Key, ConfigDraw* )>;
			using itemdropped_event = std::function<void(S *, ItemDropped, ConfigDraw* )>;

			std::list<move_event> callbacks_move;
			std::list<press_event> callbacks_press;
			std::list<scroll_event> callbacks_scroll;
			std::list<unpress_event> callbacks_unpress;
			std::list<key_event> callbacks_key;
			std::list<itemdropped_event> callbacks_itemdropped;

			void add_move_call(move_event &&call)
			{
				callbacks_move.emplace_back(std::move(call));
			}
			void add_press_call(press_event &&call)
			{
				callbacks_press.emplace_back(std::move(call));
			}
			void add_scroll_call(scroll_event &&call)
			{
				callbacks_scroll.emplace_back(std::move(call));
			}

			void add_unpress_call(unpress_event &&call)
			{
				callbacks_unpress.emplace_back(std::move(call));
			}
			void add_key_call(key_event &&call)
			{
				callbacks_key.emplace_back(std::move(call));
			}
			void add_itemdrop_call(itemdropped_event &&call)
			{
				callbacks_itemdropped.emplace_back(std::move(call));
			}
		};

    }
}

#endif 