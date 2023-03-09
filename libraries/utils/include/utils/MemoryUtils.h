#ifndef CURAN_MEMORY_UTILS_HEADER_FILE_
#define CURAN_MEMORY_UTILS_HEADER_FILE_

#include <functional>
// Exclude rarely-used stuff from Windows headers
#define WIN32_LEAN_AND_MEAN             
// Windows Header Files
#include <windows.h>
#include <asio.hpp>

namespace curan {
	namespace utils {

		/*
		Parent class which definies the characteristics required to obey the ConstBufferSequence
		*/
		class memory_buffer {
		public:
			// Implement the ConstBufferSequence requirements.
			typedef asio::const_buffer value_type;
			typedef const asio::const_buffer* const_iterator;

			virtual const asio::const_buffer* begin() const = 0;
			virtual const asio::const_buffer* end() const = 0;

			friend std::ostream& operator << (std::ostream& os,const std::shared_ptr<memory_buffer>& val);
		};

		std::ostream& operator << (std::ostream& os, const  std::shared_ptr<memory_buffer>& val);

		/*
		Typically a binding will capture the object we wish to send (to guarantee the lifetime of the object)
		and it returns a const_buffer to whichever memory we wish to send to an external socket. When dealing with
		variables which do not behave as a shared pointer use the copy memory buffer instead of the capture memory
		buffer.
		*/
		//using binding = std::function<asio::const_buffer()>;
		typedef std::function<asio::const_buffer()> binding;

		/*
		The capture memory buffer takes a lamda which constructs the view of a buffer and returns it.
		This type of object should be used when one wishes to avoid uncessary copies, and instead wishes
		send a lambda which captures the object (because it is shared) and it constructs it when required
		by the assyncronous operation.
		*/
		class capture_memory_buffer : public memory_buffer {
			// Construct from a std::string.
			explicit capture_memory_buffer(std::function<asio::const_buffer()>&& val);

		public:
			static std::shared_ptr<memory_buffer> make_shared(binding&& val);

			const asio::const_buffer* begin() const override { return &buffer_; }
			const asio::const_buffer* end() const override { return &buffer_ + 1; }

		private:
			binding val_;
			asio::const_buffer buffer_;
		};

		/*
		The copy memory buffer takes a pointer and a size of a pointer, internaly makes
		a copy of said region of memory, and then it takes ownership of said memory.
		Because we
		*/
		class copy_memory_buffer : public memory_buffer {

			// Construct from a std::string.
			explicit copy_memory_buffer(char* data, size_t size);
		public:
			static std::shared_ptr<memory_buffer> make_shared(char* data, size_t size);

			const asio::const_buffer* begin() const override { return &buffer_; }
			const asio::const_buffer* end() const override { return &buffer_ + 1; }

		private:
			std::unique_ptr<std::vector<char>> data_;
			asio::const_buffer buffer_;
		};
	}
}

#endif