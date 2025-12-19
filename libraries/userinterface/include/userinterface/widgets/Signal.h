#ifndef CURAN_SIGNAL_HEADER_FILE_
#define CURAN_SIGNAL_HEADER_FILE_

#include <string>
#include <vector>
#include <list>
#include <functional>
#include "definitions/UIdefinitions.h"
#include <ostream>
#include "utils/Overloading.h"
#include <variant>

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

		struct Press{
			enum click{
				LEFT,
				RIGHT
			};
			click button;
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

		struct Unpress{
			enum click{
				LEFT,
				RIGHT
			};
			click button;
			double xpos;
			double ypos;
		};

		enum ControlKeys
		{
			_NOT,
			_ESCAPE,
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

		static inline ControlKeys convert_to_control_key(const Key &in)
		{
			switch (in.key)
			{
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

		static inline char convert_to_acsii(const Key &in)
		{
			switch (in.key)
			{
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

		enum InterpreterStatus : size_t
		{
			MOUSE_CLICKED_LEFT_EVENT = 1 << 1,
			MOUSE_CLICKED_LEFT = 1 << 2,
			MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED = 1 << 3,
			MOUSE_CLICKED_RIGHT_EVENT = 1 << 4,
			MOUSE_CLICKED_RIGHT = 1 << 5,
			MOUSE_CLICKED_RIGHT_WAS_INSIDE_FIXED = 1 << 6,
			MOUSE_UNCLICK_LEFT_EVENT = 1 << 7,
			MOUSE_UNCLICK_RIGHT_EVENT = 1 << 8,
			MOUSE_MOVE_EVENT = 1 << 9, // move is always an event
			SCROLL_EVENT = 1 << 10,	   // scroll is always an event
			OUTSIDE_ALLOCATED_AREA = 1 << 11,
			INSIDE_ALLOCATED_AREA = 1 << 12,
			ENTERED_ALLOCATED_AREA_EVENT = 1 << 13,
			LEFT_ALLOCATED_AREA_EVENT = 1 << 14,
			OUTSIDE_FIXED_AREA = 1 << 15,
			INSIDE_FIXED_AREA = 1 << 16,
			LEFT_FIXED_AREA_EVENT = 1 << 17,
			ENTERED_FIXED_AREA_EVENT = 1 << 18,
			ITEM_DROPPED_EVENT = 1 << 19,
			KEYBOARD_EVENT = 1 << 20,
			HEART_BEAT = 1 << 21
		};

		/*
		The signal interpreter allows us to retain state
		and impose logic about how to system should behave when
		transitions between these states happen. This should
		compact most logic of widgets, since all state is
		computed inside this interpreter
		*/
		class SignalInterpreter
		{
			size_t current_status = OUTSIDE_ALLOCATED_AREA | OUTSIDE_FIXED_AREA;
			bool long_format = false;

			double x_last_move = 0.0;
			double y_last_move = 0.0;

			double x_last_press = 0.0;
			double y_last_press = 0.0;

		public:
			SignalInterpreter()
			{
			}

			inline bool check(size_t mask){
				return (current_status & mask ) == mask;
			}

		private:
			inline void shutoff_oneoff_events()
			{
				// all events are one off, meaning that when we receive a new signal,
				// we must shut them off so that we guarantee that the class does not
				// repeat behavior
				current_status &= ~HEART_BEAT;
				current_status &= ~MOUSE_CLICKED_LEFT_EVENT;
				current_status &= ~MOUSE_CLICKED_RIGHT_EVENT;
				current_status &= ~ITEM_DROPPED_EVENT;
				current_status &= ~LEFT_FIXED_AREA_EVENT;
				current_status &= ~ENTERED_FIXED_AREA_EVENT;
				current_status &= ~ENTERED_ALLOCATED_AREA_EVENT;
				current_status &= ~LEFT_ALLOCATED_AREA_EVENT;
				current_status &= ~SCROLL_EVENT;
				current_status &= ~MOUSE_MOVE_EVENT;
				current_status &= ~MOUSE_UNCLICK_RIGHT_EVENT;
				current_status &= ~MOUSE_UNCLICK_LEFT_EVENT;
				current_status &= ~KEYBOARD_EVENT;
			}

			template <typename allocated>
			inline void allocated_area_logic(allocated &&check_allocated, const double x, const double y)
			{
				if (check_allocated(x, y)) // if inside allocated area
				{
					if (current_status & OUTSIDE_ALLOCATED_AREA)
					{
						current_status |= ENTERED_ALLOCATED_AREA_EVENT;
					}
					current_status &= ~OUTSIDE_ALLOCATED_AREA;
					current_status |= INSIDE_ALLOCATED_AREA;
				}
				else if (current_status & INSIDE_ALLOCATED_AREA)
				{
					current_status &= ~INSIDE_ALLOCATED_AREA;
					current_status |= LEFT_ALLOCATED_AREA_EVENT | OUTSIDE_ALLOCATED_AREA;
				}
			}

			template <typename fixed>
			inline void fixed_area_logic(fixed &&check_fixed, const double x, const double y)
			{
				if (check_fixed(x, y)) // if inside allocated area
				{
					if (current_status & OUTSIDE_FIXED_AREA)
					{
						current_status |= ENTERED_FIXED_AREA_EVENT;
					}
					current_status &= ~OUTSIDE_FIXED_AREA;
					current_status |= INSIDE_FIXED_AREA;
				}
				else if (current_status & INSIDE_FIXED_AREA)
				{
					current_status &= ~INSIDE_FIXED_AREA;
					current_status |= LEFT_FIXED_AREA_EVENT | OUTSIDE_FIXED_AREA;
				}
			}

		public:
			template <typename allocated, typename fixed>
			void process(allocated &&inside_allocated, fixed &&inside_fixed, curan::ui::Signal current_signal)
			{
				std::visit(curan::utilities::overloaded{[&](curan::ui::Empty arg)
														{
															shutoff_oneoff_events();
															current_status |= HEART_BEAT;
														},
														[&](curan::ui::Move arg)
														{
															shutoff_oneoff_events();
															allocated_area_logic(std::forward<allocated>(inside_allocated), arg.xpos, arg.ypos);
															fixed_area_logic(std::forward<fixed>(inside_fixed), arg.xpos, arg.ypos);
															current_status |= MOUSE_MOVE_EVENT;
															x_last_move = arg.xpos;
															y_last_move = arg.ypos;
														},
														[&](curan::ui::Press arg)
														{
															shutoff_oneoff_events();
															allocated_area_logic(std::forward<allocated>(inside_allocated), arg.xpos, arg.ypos);
															fixed_area_logic(std::forward<fixed>(inside_fixed), arg.xpos, arg.ypos);
															if(arg.button==curan::ui::Press::LEFT){
																if (!(current_status & MOUSE_CLICKED_LEFT)){
																	if(current_status & INSIDE_FIXED_AREA){
																		current_status |= MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED;
																	}
																		
																	current_status |= MOUSE_CLICKED_LEFT_EVENT | MOUSE_CLICKED_LEFT;
																}
															} else {
																if (!(current_status & MOUSE_CLICKED_RIGHT)){
																	if(current_status & INSIDE_FIXED_AREA)
																		current_status |= MOUSE_CLICKED_RIGHT_WAS_INSIDE_FIXED;
																	current_status |= MOUSE_CLICKED_RIGHT_EVENT | MOUSE_CLICKED_RIGHT;
																}
															}



															x_last_press = arg.xpos;
															y_last_press = arg.ypos;
														},
														[&](curan::ui::Scroll arg)
														{
															shutoff_oneoff_events();
															allocated_area_logic(std::forward<allocated>(inside_allocated), arg.xpos, arg.ypos);
															fixed_area_logic(std::forward<fixed>(inside_fixed), arg.xpos, arg.ypos);
															current_status |= SCROLL_EVENT;
														},
														[&](curan::ui::Unpress arg)
														{
															shutoff_oneoff_events();
															allocated_area_logic(std::forward<allocated>(inside_allocated), arg.xpos, arg.ypos);
															fixed_area_logic(std::forward<fixed>(inside_fixed), arg.xpos, arg.ypos);
															if(arg.button==curan::ui::Unpress::LEFT){
																if ((current_status & MOUSE_CLICKED_LEFT))
																{
																	current_status |= MOUSE_UNCLICK_LEFT_EVENT;
																	current_status &= ~MOUSE_CLICKED_LEFT;
																}
															} else {
																if ((current_status & MOUSE_CLICKED_RIGHT))
																{
																	current_status |= MOUSE_UNCLICK_RIGHT_EVENT;
																	current_status &= ~MOUSE_CLICKED_RIGHT;
																}
															}
															current_status &= ~(MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED | MOUSE_CLICKED_RIGHT_WAS_INSIDE_FIXED);
														},
														[&](curan::ui::Key arg)
														{
															shutoff_oneoff_events();
															current_status |= KEYBOARD_EVENT;
														},
														[&](curan::ui::ItemDropped arg)
														{
															shutoff_oneoff_events();
															current_status |= ITEM_DROPPED_EVENT;
														}},
						   current_signal);
			};

			inline size_t status()
			{
				return current_status;
			}

			inline void set_format(bool val)
			{
				long_format = val;
			}

			inline std::pair<double, double> last_press() const
			{
				return std::make_pair(x_last_press, y_last_press);
			}

			inline std::pair<double, double> last_move() const
			{
				return std::make_pair(x_last_move, y_last_move);
			}

			friend std::ostream &operator<<(std::ostream &o, const SignalInterpreter &val)
			{

				o << "status: " << std::bitset<sizeof(size_t) * 8>{val.current_status} << std::endl;

				if (!val.long_format)
					return o;

				if (val.current_status & MOUSE_CLICKED_LEFT_EVENT)
				{
					o << "MOUSE_CLICKED_LEFT_EVENT\n";
				}

				if (val.current_status & MOUSE_CLICKED_LEFT)
				{
					o << "MOUSE_CLICKED_LEFT\n";
				}

				if (val.current_status & MOUSE_CLICKED_RIGHT_EVENT)
				{
					o << "MOUSE_CLICKED_RIGHT_EVENT\n";
				}

				if (val.current_status & MOUSE_CLICKED_RIGHT)
				{
					o << "MOUSE_CLICKED_RIGHT\n";
				}

				if (val.current_status & MOUSE_UNCLICK_LEFT_EVENT)
				{
					o << "MOUSE_UNCLICK_LEFT_EVENT\n";
				}

				if (val.current_status & MOUSE_UNCLICK_RIGHT_EVENT)
				{
					o << "MOUSE_UNCLICK_RIGHT_EVENT\n";
				}

				if(val.current_status & MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED)
				{
					o << "MOUSE_CLICKED_LEFT_WAS_INSIDE_FIXED\n";
				}

				if(val.current_status & MOUSE_CLICKED_RIGHT_WAS_INSIDE_FIXED)
				{
					o << "MOUSE_CLICKED_RIGHT_WAS_INSIDE_FIXED\n";
				}

				if (val.current_status & MOUSE_MOVE_EVENT)
				{
					o << "MOUSE_MOVE_EVENT\n";
				}

				if (val.current_status & SCROLL_EVENT)
				{
					o << "SCROLL_EVENT\n";
				}

				if (val.current_status & OUTSIDE_ALLOCATED_AREA)
				{
					o << "OUTSIDE_ALLOCATED_AREA\n";
				}

				if (val.current_status & INSIDE_ALLOCATED_AREA)
				{
					o << "INSIDE_ALLOCATED_AREA\n";
				}

				if (val.current_status & ENTERED_ALLOCATED_AREA_EVENT)
				{
					o << "ENTERED_ALLOCATED_AREA_EVENT\n";
				}

				if (val.current_status & LEFT_ALLOCATED_AREA_EVENT)
				{
					o << "LEFT_ALLOCATED_AREA_EVENT\n";
				}

				if (val.current_status & OUTSIDE_FIXED_AREA)
				{
					o << "OUTSIDE_FIXED_AREA\n";
				}

				if (val.current_status & INSIDE_FIXED_AREA)
				{
					o << "INSIDE_FIXED_AREA\n";
				}

				if (val.current_status & LEFT_FIXED_AREA_EVENT)
				{
					o << "LEFT_FIXED_AREA_EVENT\n";
				}

				if (val.current_status & ENTERED_FIXED_AREA_EVENT)
				{
					o << "ENTERED_FIXED_AREA_EVENT\n";
				}

				if (val.current_status & ITEM_DROPPED_EVENT)
				{
					o << "ITEM_DROPPED_EVENT\n";
				}

				if (val.current_status & KEYBOARD_EVENT)
				{
					o << "KEYBOARD_EVENT\n";
				}

				if (val.current_status & HEART_BEAT)
				{
					o << "HEART_BEAT\n";
				}
				return o;
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