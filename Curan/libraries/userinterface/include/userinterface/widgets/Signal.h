#ifndef CURAN_SIGNAL_HEADER_FILE_
#define CURAN_SIGNAL_HEADER_FILE_

#include <variant>
#include <string>
#include <vector>

#include "definitions/UIdefinitions.h"
#include "utils/Overloading.h"

namespace curan
{
	namespace ui
	{
		struct Empty
		{
		};

		struct Move
		{
			double xpos;
			double ypos;
		};

		struct Press
		{
			double xpos;
			double ypos;
		};

		struct Scroll
		{
			double xpos;
			double ypos;
			double xoffset;
			double yoffset;
		};

		struct Unpress
		{
			double xpos;
			double ypos;
		};

		struct Key
		{
			char pressed_char;
		};

		struct ItemDropped
		{
			int count;
			std::vector<std::string> paths;
		};

		using Signal = std::variant<Empty, Move, Press, Scroll, Unpress, ItemDropped, Key>;

		template <typename widget>
		struct SignalProcessor
		{
			using callback_move = std::function<void(widget *, Move);
			using callback_press = std::function<void(widget *, Press)>;
			using callback_scroll = std::function<void(widget *, Scroll)>;
			using callback_unpress = std::function<void(widget *, Unpress)>;
			using callback_key = std::function<void(widget *, Key)>;
			using callback_itemdropped = std::function<void(widget *, ItemDropped)>;

			std::list<callback_move> callbacks_move;
			std::list<callback_press> callbacks_press;
			std::list<callback_scroll> callbacks_scroll;
			std::list<callback_unpress> callbacks_unpress;
			std::list<callback_key> callbacks_key;
			std::list<callback_itemdropped> callbacks_itemdropped;

			void add_move_call(callback_move &&call)
			{
				callbacks_move.emplace_back(std::move(call));
			}
			void add_press_call(callback_press &&call)
			{
				callbacks_press.emplace_back(std::move(call));
			}
			void add_scroll_call(callback_scroll &&call)
			{
				callbacks_scroll.emplace_back(std::move(call));
			}

			void add_unpress_call(callback_unpress &&call)
			{
				callbacks_unpress.emplace_back(std::move(call));
			}
			void add_key_call(callback_key &&call)
			{
				callbacks_key.emplace_back(std::move(call));
			}
			void add_itemdrop_call(callback_itemdropped &&call)
			{
				callbacks_itemdropped.emplace_back(std::move(call));
			}
		};

		void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);

		void cursor_position_click_callback(GLFWwindow *window, int button, int action, int mods);

		void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

		void item_droped_callback(GLFWwindow *window, int count, const char **paths);

		void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
	}
}

#endif