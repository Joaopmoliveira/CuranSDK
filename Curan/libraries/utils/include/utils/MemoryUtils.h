#ifndef CURAN_MEMORY_UTILS_HEADER_FILE_
#define CURAN_MEMORY_UTILS_HEADER_FILE_

#include <functional>
// Exclude rarely-used stuff from Windows headers
#ifdef CURAN_WINDOWS             
// Windows Header Files
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <asio.hpp>

namespace curan {
namespace utilities {

/*
The MemoryBuffer class is a pivotal class inside Curan. 
It allows us to move memory around 
and customize the behavior of copying said memory 
depending on the desired behavior of the overall application.
Under the hood, the class used ASIO API to define buffers for two
reasons, given how widespread ASIO is, we acknowledge that its
API to represent memory regions is flexible and robust. Because
ASIO is cheap, and a header library, we believe adding this dependency
to such a fundamental library of Curan as utils is not problematic.

The class MemoryBuffer is abstract, meaning that it cannot be instantiated,
thus children's classes must implement whatever custom functionality we desire. Internally the class
can be iterated, given the definition of a begin and end virtual calls. The << operator is also defined 
so that we can print buffers for debug purposes. 

There are two children of the parent MemoryBuffer:
1) CaptureBuffer
2) CopyBuffer

Which serve very distinct purposes. 

The CaptureBuffer has a templated constructor, 
which receives a generic template argument defining
 an object which controls the lifetime of the underlying memory
*/

class MemoryBuffer {
public:
typedef asio::const_buffer value_type;
typedef const asio::const_buffer* const_iterator;

virtual const asio::const_buffer* begin() const = 0;
virtual const asio::const_buffer* end() const = 0;

friend std::ostream& operator << (std::ostream& os,const std::shared_ptr<MemoryBuffer>& val);
};

std::ostream& operator << (std::ostream& os, const  std::shared_ptr<MemoryBuffer>& val);

typedef std::function<asio::const_buffer()> binding;

class CaptureBuffer final : public MemoryBuffer
{
template <typename T>
explicit CaptureBuffer(void* buff,size_t size,T u):_u{u}{
	buffer_ = asio::buffer(buff,size);
}

public:

template <typename T>
static std::shared_ptr<MemoryBuffer> make_shared(void* in_buff,size_t size,T u){
    std::shared_ptr<CaptureBuffer> buff = std::shared_ptr<CaptureBuffer>{ new CaptureBuffer{in_buff,size,u} };
	return buff;
}

inline const asio::const_buffer *begin() const override { return &buffer_; }
inline const asio::const_buffer *end() const override { return &buffer_ + 1; }

private:
std::any _u;
asio::const_buffer buffer_;
};

class CopyBuffer final : public MemoryBuffer {
explicit CopyBuffer(char* data, size_t size);
public:
static std::shared_ptr<MemoryBuffer> make_shared(char* data, size_t size);

inline const asio::const_buffer* begin() const override { return &buffer_; }
inline const asio::const_buffer* end() const override { return &buffer_ + 1; }

private:
std::unique_ptr<std::vector<char>> data_;
asio::const_buffer buffer_;
};

}
}

#endif