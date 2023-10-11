#ifndef CURAN_SIGNAL_HEADER_FILE_
#define CURAN_SIGNAL_HEADER_FILE_

#include <variant>
#include <string>
#include <vector>
#include <list>
#include <functional>
#include "definitions/UIdefinitions.h"

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

		void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);

		void cursor_position_click_callback(GLFWwindow *window, int button, int action, int mods);

		void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

		void item_droped_callback(GLFWwindow *window, int count, const char **paths);

		void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
	}
}

#endif