#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoProcHandler.h"

#include "utils/Flag.h"
#include "utils/TheadPool.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include <cmath>
#include <csignal>
#include <system_error>

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Loader.h"
#include "utils/Logger.h"
#include "utils/Overloading.h"
#include <variant>

#include <iostream>
#include <thread>

#ifdef CURAN_WINDOWS
#include <tchar.h>
#include <windows.h>
#elif CURAN_LINUX
#define _OPEN_SYS
#include <sys/wait.h>
#include <unistd.h>
#include <sys/types.h>
#endif

#include <stdio.h>
#include <filesystem>

asio::io_context* ptr_ctx = nullptr;

void signal_handler(int signal)
{
	if (ptr_ctx) ptr_ctx->stop();
}

struct PlatformAgnosticCmdArgs{
	
#ifdef CURAN_WINDOWS
		std::string cmd;

		PlatformAgnosticCmdArgs(const std::string& in_cmd) : cmd{in_cmd} {}

#elif CURAN_LINUX
		std::vector<std::string> cmd;

		PlatformAgnosticCmdArgs(const std::vector<std::string>& in_cmd) : cmd{in_cmd} {}
#endif 
};

std::ostream& operator<< (std::ostream& o, PlatformAgnosticCmdArgs args){
#ifdef CURAN_WINDOWS
	o << args.cmd;
#elif CURAN_LINUX	
	std::vector<std::string> cmd;
	for(const auto& val : args.cmd)
		o << val << " ";
#endif 	
	return o;
}

class ProcessHandles {
#ifdef CURAN_WINDOWS
	PROCESS_INFORMATION pi;
#elif CURAN_LINUX
	pid_t pi = 0;
#endif
public:
	ProcessHandles() {
#ifdef CURAN_WINDOWS
	ZeroMemory(&pi, sizeof(pi));
#elif CURAN_LINUX
	pi = 0;
#endif	
	}

	operator bool() const {
#ifdef CURAN_WINDOWS
		PROCESS_INFORMATION local;
		ZeroMemory(&local, sizeof(local));
		// memcmp(&local, &pi, sizeof(local)); returns 0 if all bytes are equal, i.e., if the memory of pi is zeroed out. If it is zeroed then it mean that the 
		// handles are closed, if they are different then returns number different from zero
		return memcmp(&local, &pi, sizeof(local)); // compare if the handles are nullified

#elif CURAN_LINUX
		return pi;
#endif 
	}

	template<class _Rep, class _Period>
	void close(const std::chrono::duration<_Rep, _Period>& deadline) {
		if (!(*this)) {
			return;
		}
		
#ifdef CURAN_WINDOWS
		std::chrono::milliseconds transformed_deadline = std::chrono::duration_cast<std::chrono::milliseconds>(deadline);
		auto return_value = WaitForSingleObject(pi.hProcess, transformed_deadline.count());
		switch (return_value) {
		case WAIT_ABANDONED: //this should never happen? it happears to be related with mutex handles
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_OBJECT_0: // this means that the operation was successeful
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_TIMEOUT: // the wait operation timed out thus unsuccesefull
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		case WAIT_FAILED: // the wait failed for obscure reasons
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		default:
			TerminateProcess(pi.hProcess, 3);
			CloseHandle(pi.hProcess);
			CloseHandle(pi.hThread);
			ZeroMemory(&pi, sizeof(pi));
			break;
		}
		// if the waiting operation does not terminate in the alloted time we force the process to terminate manually

#elif CURAN_LINUX
		int status = -10;
		int vals = -10;
		// the behavior is different if we need to wait for a larger amount of time than 0 or if it is zero
		std::chrono::nanoseconds transformed_deadline = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		if(transformed_deadline.count() == 0){
			vals = waitpid(pi, &status, WNOHANG);
        	if (vals == -1) {
            	// failure to wait, (basically the wait itself could not run to check on the child process)
				// I think I should still try to kill the proccess in this case
				kill(pi,SIGINT);
				pi = 0;
       	 	}

        	if (vals>0) { // this is true, it means that the wait was suceesefull and we can anlyse the status flag to see the status of the other process
            	if (WIFEXITED(status)) { // if this is true the other process is closed, so we don't need to do anything
                	pi = 0;
            	} else { // if this is triggered then the process was terminated for some other reason, still don't know if I should kill it in this situation or not
					pi = 0;
				}
        	} else if(pi){ // the wait did not finish, meaning that the process is still active,
				kill(pi,SIGINT);
				pi = 0;
			}

		} else {
			auto start_wait_time = std::chrono::high_resolution_clock::now();
			auto current_time = std::chrono::high_resolution_clock::now();
			auto copy_of_pid = pi;
    		do {
				vals = waitpid(copy_of_pid, &status, WNOHANG);
        		if (vals == -1) {
            		// failure to wait, (basically the wait itself could not run to check on the child process)
					// I think I should still try to kill the proccess in this case 
       	 		}

        		if (vals) { // this is true, it means that the wait was suceesefull and we can anlyse the status flag to see the status of the other process
            		if (WIFEXITED(status)) { // if this is true the other process is closed, so we don't need to do anything
						copy_of_pid = 0;
						break;
            		} else { // if this is triggered then the process was terminated for some other reason, still don't know if I should kill it in this situation or not
						copy_of_pid = 0;
						break;
					}
        		} else { // the wait did not finish, meaning that the process is still active,
				}
				std::this_thread::sleep_for(transformed_deadline/10);
				current_time = std::chrono::high_resolution_clock::now();
				
    		} while (std::chrono::duration_cast<std::chrono::nanoseconds>(current_time-start_wait_time)<transformed_deadline);
			if(copy_of_pid){
				kill(pi,SIGINT);
				pi = 0;
			}
		}


#endif // CURAN_WINDOWS

	}

	bool open(PlatformAgnosticCmdArgs& s) {
#ifdef CURAN_WINDOWS
		STARTUPINFO si;

		ZeroMemory(&si, sizeof(si));
		si.cb = sizeof(si);
		ZeroMemory(&pi, sizeof(pi));
		TCHAR* val = s.cmd.data();
		//const TCHAR* data = static_cast<const TCHAR*>(command.data());
		;
		// Start the child process. 
		if (!CreateProcess(NULL,   // No module name (use command line)
			val,        // Command line
			NULL,           // Process handle not inheritable
			NULL,           // Thread handle not inheritable
			FALSE,          // Set handle inheritance to FALSE
			0,              // No creation flags
			NULL,           // Use parent's environment block
			NULL,           // Use parent's starting directory 
			&si,            // Pointer to STARTUPINFO structure
			&pi)           // Pointer to PROCESS_INFORMATION structure
			) {
			return false;
		}
		return true;

#elif CURAN_LINUX
		pi = fork();
		if(pi < 0 ){
			return false;
		} 
		else if (pi > 0)
		{
			return true;
		}
		else 
		{		
			std::vector<char*> buffer_to_pass;
			for(auto& arg : s.cmd)
				buffer_to_pass.push_back(arg.data());
			buffer_to_pass.push_back(NULL);
    		execve(s.cmd.at(0).data(),buffer_to_pass.data(),NULL);
    		_exit(EXIT_FAILURE);   // exec never returns
		}
		return false;
#endif 
	}

};



template<typename... Args>
PlatformAgnosticCmdArgs create_command(unsigned short port, Args ... arg) {
	constexpr size_t size = sizeof ...(Args);
	const char* loc[size] = { arg... };
	std::string command;

	//first we check if the executable exists in the current computer
	std::string executable_directory_sanity_check{loc[0]};
	std::filesystem::path path{ executable_directory_sanity_check };

	if (!std::filesystem::exists(path))
		throw std::runtime_error("the specified file does not exist");

#ifdef CURAN_WINDOWS
	std::string cmd;

	// now depending if we are on linux or windows we need distinct behavior	
	for (const auto& val : loc)
		cmd += std::string(val) + " ";

	cmd += std::to_string(port);	

	PlatformAgnosticCmdArgs args{cmd};

#elif CURAN_LINUX
	std::vector<std::string> cmd;

	// now depending if we are on linux or windows we need distinct behavior	
	for (const auto& val : loc)
		cmd.push_back(std::string(val));;

	cmd.push_back(std::to_string(port));

	PlatformAgnosticCmdArgs args{cmd};

#endif 
	return args;
}

class ProcessLaucher {

	asio::io_context& hidden_context;
	asio::high_resolution_timer connection_timer;
	asio::high_resolution_timer closing_process_timer;
	std::chrono::nanoseconds duration;
	size_t number_of_violations;
	bool was_violated;
	const size_t max_num_violations;
	std::shared_ptr<curan::communication::Server> server;
	bool connection_established = false;
	unsigned short port;

public:

	ProcessHandles handles;

	template <class _Rep, class _Period>
	ProcessLaucher(asio::io_context& client_ctx, const std::chrono::duration<_Rep, _Period>& deadline, size_t max_viols, unsigned short in_port = 50000) :
		hidden_context{ client_ctx },
		connection_timer{ client_ctx },
		closing_process_timer{ client_ctx },
		number_of_violations{ 0 },
		was_violated{ true },
		max_num_violations{ max_viols },
		port{ in_port },
		handles{}
	{
		duration = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline);
		using namespace curan::communication;
		interface_prochandler igtlink_interface;
		Server::Info construction{ client_ctx,igtlink_interface ,port };
		server = Server::make(construction,
			[this](std::error_code er) {
				if (!er && !connection_established) {
					connection_established = true;
					return true;
				} 
				if (connection_established)
					return false;
				server->cancel();
				return false;
			}
		);

		server->connect([this](const size_t& protocol_defined_val,
			const std::error_code& er,
			std::shared_ptr<curan::communication::ProcessHandler> val) {
				message_callback(protocol_defined_val, er, val);
			});

		connection_timer.expires_from_now(duration);
		connection_timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	void timer_callback(asio::error_code ec) {
		if (ec)
			return;
		if (connection_established) {
			
			number_of_violations = was_violated ? number_of_violations + 1 : 0;
			was_violated = true;
			if (number_of_violations > max_num_violations) {
				auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
				val->serialize();
				auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
				server->write(to_send);
				async_terminate(std::chrono::seconds(3),[this](){ 
					connection_timer.cancel();
					closing_process_timer.cancel();
					sync_internal_terminate_pending_process_and_connections();
					hidden_context.get_executor().on_work_finished();
				});
				return;
			}
			auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::HEART_BEAT);
			val->serialize();
			auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
			server->write(to_send);
		}
		connection_timer.expires_from_now(duration);
		connection_timer.async_wait([this](asio::error_code ec) {
			timer_callback(ec);
			});
	}

	~ProcessLaucher() {
		connection_timer.cancel();
		closing_process_timer.cancel();
		sync_internal_terminate_pending_process_and_connections();
	}

	void sync_internal_terminate_pending_process_and_connections() {
		connection_established = false;
		server->cancel();
	}

	void message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			sync_internal_terminate_pending_process_and_connections();
			return;
		}
		switch (val->signal_to_process) {
		case curan::communication::ProcessHandler::Signals::HEART_BEAT:
			was_violated = false;
			number_of_violations = 0;
			break;
		case curan::communication::ProcessHandler::Signals::SHUTDOWN_SAFELY:
		default:
			break;
		}
	}

	/*
	This function takes the arguments, appends them with a space and then adds at the end the port of the current server
	*/
	template<typename... Args>
	bool lauch_process(Args ... arg) {
		if (handles) {
			return false;
		}
		auto command = create_command(port,arg...);
		return handles.open(command);
	}

	/*
	This function takes the arguments, appends them with a space and then adds at the end the port of the current server
	*/
	template<typename... Args>
	void async_lauch_process(std::function<void(bool)> async_handler, Args ... arg) {
		if (handles) { // we cannot lauch an asycn proces without closing the previous handles
			async_handler(false);
			return;
		}
		auto command = create_command(port, arg...);
		hidden_context.post([this, command, async_handler]() mutable {
				async_handler(handles.open(command));
			}
		);

	}

	/*
	This is a blocking call which waits until the other process is stopped. 
	don't use it inside the asio executor or you block all asyncronous operations
	*/
	template <class _Rep, class _Period>
	void terminate(const std::chrono::duration<_Rep, _Period>& deadline) {
		if (!handles) //if the handles are already closed then we don't need to close anything
			return;
		
		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		server->write(to_send);

		handles.close(deadline);
	}

	template <class _Rep, class _Period>
	void async_terminate(const std::chrono::duration<_Rep, _Period>& deadline,std::function<void(void)> termination_handler) {
		if (!handles)
			return;

		auto val = std::make_shared<curan::communication::ProcessHandler>(curan::communication::ProcessHandler::SHUTDOWN_SAFELY);
		val->serialize();
		auto to_send = curan::utilities::CaptureBuffer::make_shared(val->buffer.data(), val->buffer.size(), val);
		server->write(to_send);

		closing_process_timer.expires_from_now(deadline);
		closing_process_timer.async_wait(
			[this,termination_handler](asio::error_code ec) {
				handles.close(std::chrono::milliseconds(0)); // the method attempts to check if the other process is close and then is kills the other process automatically
				termination_handler();
			}
		);

	}
};

int viewer_code() {
	try {
		using namespace curan::ui;
		IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};
		std::unique_ptr<Context> context = std::make_unique<Context>();;
		DisplayParams param{ std::move(context),2200,1800 };
		std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

	    auto button1 = Button::make("Temporal Calibration",resources);
	    button1->set_click_color(SK_ColorDKGRAY).set_hover_color(SK_ColorLTGRAY).set_waiting_color(SK_ColorGRAY).set_size(SkRect::MakeWH(300, 300));

		auto icon = resources.get_icon("hr_repeating.png");
	    auto widgetcontainer =  Container::make(Container::ContainerType::LINEAR_CONTAINER,Container::Arrangement::VERTICAL);
	    *widgetcontainer << std::move(button1);

		widgetcontainer->set_color(SK_ColorBLACK);
		auto page = Page{std::move(widgetcontainer),SK_ColorBLACK};
		page.update_page(viewer.get());

		ConfigDraw config_draw{ &page};
		config_draw.stack_page->stack(Loader::make("human_robotics_logo.jpeg",resources));

		viewer->set_minimum_size(page.minimum_size());

		while (!glfwWindowShouldClose(viewer->window)) {
			auto start = std::chrono::high_resolution_clock::now();
			SkSurface* pointer_to_surface = viewer->getBackbufferSurface();

			SkCanvas* canvas = pointer_to_surface->getCanvas();
			if (viewer->was_updated()) {
		    	page.update_page(viewer.get());
				viewer->update_processed();
			}
			page.draw(canvas);
			auto signals = viewer->process_pending_signals();

			if (!signals.empty())
				page.propagate_signal(signals.back(), &config_draw);
			page.propagate_heartbeat(&config_draw);
			glfwPollEvents();

			bool val = viewer->swapBuffers();
			if (!val)
				std::cout << "failed to swap buffers\n";
			auto end = std::chrono::high_resolution_clock::now();
			std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
		}
		return 0;
	}
	catch (std::exception& e) {
		std::cout << "Failed: " << e.what() << std::endl;
		return 1;
	}
}

int main() {
	try {
		using namespace curan::communication;
		std::signal(SIGINT, signal_handler);
		asio::io_context io_context;
		ptr_ctx = &io_context;
		auto parent = std::make_unique<ProcessLaucher>(io_context, std::chrono::milliseconds(100), 10);
		parent->async_lauch_process([](bool sucess) {  }, CURAN_BINARY_LOCATION"/mimic_child_proc" CURAN_BINARY_SUFFIX);
		io_context.run();
	}
	catch (std::exception& e) {
		return 1;
	}
	catch (...) {
		return 1;
	}
	return 0;
}