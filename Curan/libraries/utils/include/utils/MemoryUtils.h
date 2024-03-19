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