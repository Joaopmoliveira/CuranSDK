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

		enum ControlKeys{
			_NOT,
			_ESCAPE ,
			_ENTER,
			_TAB,
			_BACKSPACE,
			_INSERT,
			_DELETE,
			_RIGHT,
			_LEFT,
			_DOWN,
			_UP,
			_PAGE_UP,
			_PAGE_DOWN
		};

		struct Key
		{
			int key;
			int scancode;
			int action; 
			int mods;
			ControlKeys key_type;
			char ascii_version;
		};

		static inline ControlKeys convert_to_control_key(const Key& in){
			switch(in.key){
				case GLFW_KEY_ESCAPE:
					return ControlKeys::_ESCAPE;
				case GLFW_KEY_ENTER:
					return ControlKeys::_ENTER;
				case GLFW_KEY_TAB:
					return ControlKeys::_TAB;
				case GLFW_KEY_BACKSPACE:
					return ControlKeys::_BACKSPACE;
				case GLFW_KEY_INSERT:
					return ControlKeys::_INSERT;
				case GLFW_KEY_DELETE:
					return ControlKeys::_DELETE;
				case GLFW_KEY_RIGHT:
					return ControlKeys::_RIGHT;
				case GLFW_KEY_LEFT:
					return ControlKeys::_LEFT;
				case GLFW_KEY_DOWN:
					return ControlKeys::_DOWN;
				case GLFW_KEY_UP:
					return ControlKeys::_UP;
				case GLFW_KEY_PAGE_UP:
					return ControlKeys::_PAGE_UP;
				case GLFW_KEY_PAGE_DOWN:
					return ControlKeys::_PAGE_DOWN;
				default:
					return ControlKeys::_NOT;
			}
		}

		static inline char convert_to_acsii(const Key& in){
			switch(in.key){
				case GLFW_KEY_SPACE:
					return ' ';
				case GLFW_KEY_APOSTROPHE:
					return '\'';
				case GLFW_KEY_COMMA:
					return ',';
				case GLFW_KEY_MINUS:
					return '-';
				case GLFW_KEY_PERIOD:
					return '.';
				case GLFW_KEY_SLASH:
					return '/';
				case GLFW_KEY_0:
					return '0';
				case GLFW_KEY_1:
					return '1';
				case GLFW_KEY_2:
					return '2';
				case GLFW_KEY_3:
					return '3';
				case GLFW_KEY_4:
					return '4';
				case GLFW_KEY_5:
					return '5';
				case GLFW_KEY_6:
					return '6';
				case GLFW_KEY_7:
					return '7';
				case GLFW_KEY_8:
					return '8';
				case GLFW_KEY_9:
					return '9';
				case GLFW_KEY_SEMICOLON:
					return ':';
				case GLFW_KEY_EQUAL:
					return '=';
				case GLFW_KEY_A:
					return 'a';
				case GLFW_KEY_B:
					return 'b';
				case GLFW_KEY_C:
					return 'c';
				case GLFW_KEY_D:
					return 'd';
				case GLFW_KEY_E:
					return 'e';
				case GLFW_KEY_F:
					return 'f';
				case GLFW_KEY_G:
					return 'g';
				case GLFW_KEY_H:
					return 'h';
				case GLFW_KEY_I:
					return 'i';
				case GLFW_KEY_J:
					return 'j';
				case GLFW_KEY_K:
					return 'k';
				case GLFW_KEY_L:
					return 'l';
				case GLFW_KEY_M:
					return 'm';
				case GLFW_KEY_N:
					return 'n';
				case GLFW_KEY_O:
					return 'o';
				case GLFW_KEY_P:
					return 'p';
				case GLFW_KEY_Q:
					return 'q';
				case GLFW_KEY_R:
					return 'r';
				case GLFW_KEY_S:
					return 's';
				case GLFW_KEY_T:
					return 't';
				case GLFW_KEY_U:
					return 'u';
				case GLFW_KEY_V:
					return 'v';
				case GLFW_KEY_W:
					return 'w';
				case GLFW_KEY_X:
					return 'x';
				case GLFW_KEY_Y:
					return 'y';
				case GLFW_KEY_Z:
					return 'z';
				case GLFW_KEY_LEFT_BRACKET:
					return '[';
				case GLFW_KEY_RIGHT_BRACKET:
					return ']';
				case GLFW_KEY_BACKSLASH:
					return '\\';
				default:
					return ' ';
			};
		}

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