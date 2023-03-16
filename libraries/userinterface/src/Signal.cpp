#include "userinterface/widgets/Signal.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include "userinterface/Window.h"

namespace curan {
	namespace ui {
		void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
			Move data{ xpos,ypos };
			Signal received_signal = data;
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		}

		void cursor_position_click_callback(GLFWwindow* window, int button, int action, int mods) {
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			Signal received_signal;
			switch (action) {
			case GLFW_PRESS:
			{
				Press val{ xpos,ypos };
				received_signal = val;
			}
			break;
			case GLFW_RELEASE:
			{
				Unpress val{ xpos,ypos };
				received_signal = val;

			}
			break;
			}
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		}

		void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			Signal received_signal;
			Scroll val{ xpos,ypos,xoffset,yoffset };
			received_signal = val;
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		}

		void item_droped_callback(GLFWwindow* window, int count, const char** paths) {
			ItemDropped dropped;
			dropped.count = count;
			dropped.paths.resize(dropped.count);
			for (int index = 0; index < count; ++index) {
				std::string path_i{ paths[index] };
				dropped.paths[index] = path_i;
			}
			Signal received_signal = dropped;
			auto window_pointer = reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
			window_pointer->signal_queue.push(received_signal);
		}
	}
}