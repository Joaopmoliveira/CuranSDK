#include "processmanagement/SingleChildParent.h"

namespace curan{
namespace process{

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

	ProcessHandles::ProcessHandles() {
#ifdef CURAN_WINDOWS
	ZeroMemory(&pi, sizeof(pi));
#elif CURAN_LINUX
	pi = 0;
#endif	
	}

	ProcessHandles::~ProcessHandles(){
		close(std::chrono::milliseconds(0));
	}

	ProcessHandles::operator bool() const {
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

	bool ProcessHandles::open(PlatformAgnosticCmdArgs& s) {
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
			CREATE_NEW_CONSOLE ,              // No creation flags
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


	void ProcessLaucher::timer_callback(asio::error_code ec) {
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
					
					sync_internal_terminate_pending_process_and_connections(CLIENT_FAILURE_MAX_FAILED_HEARBEATS);
					
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

	ProcessLaucher::~ProcessLaucher() {
		sync_internal_terminate_pending_process_and_connections(SERVER_TIMEOUT_REACHED);
	}

	void ProcessLaucher::sync_internal_terminate_pending_process_and_connections(const Failure& failure_reason ) {
		switch(failure_reason){
		case Failure::CLIENT_FAILURE_MAX_FAILED_HEARBEATS:
			connection_established = false;
			server->cancel();
			closing_process_timer.cancel();
			hidden_context.get_executor().on_work_finished();
			if(connection_callback) connection_callback(false);
		break;
		case Failure::SERVER_TIMEOUT_REACHED:
			connection_established = false;
			server->close();
			closing_process_timer.cancel();
			connection_timer.cancel();
			server_connection_timer.cancel();
			hidden_context.get_executor().on_work_finished();
			if(connection_callback) connection_callback(false);
		break;
		default:
			connection_established = false;
			server->close();
			closing_process_timer.cancel();
			server_connection_timer.cancel();
			hidden_context.get_executor().on_work_finished();
			if(connection_callback) connection_callback(false);
		break;
		}

	}

	void ProcessLaucher::message_callback(const size_t& protocol_defined_val, const std::error_code& er, std::shared_ptr<curan::communication::ProcessHandler> val) {
		if (er) {
			connection_established = false;
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

}
}