#ifndef CURAN_PROCESS_HEADER_FILE_
#define CURAN_PROCESS_HEADER_FILE_

#include <stdio.h>
#include <string>
#include <memory>

namespace curan {
namespace proc {

class Process{

void terminate();

void async_terminate();

bool terminated();

std::unique_ptr<Process> create(std::string ...);

private:

Process(std::string ...);

bool is_running;

#if CURAN_LINUX

pid_t pid_of_child = -1;

#else

#endif

};

}
}

#endif