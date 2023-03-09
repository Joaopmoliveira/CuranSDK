#include "utils/MemoryUtils.h"

namespace curan {
	namespace utils {
		capture_memory_buffer::capture_memory_buffer(binding&& val) : val_(val) {
			buffer_ = val();
		}

		std::shared_ptr<memory_buffer> capture_memory_buffer::make_shared(binding&& val) {
			std::shared_ptr<capture_memory_buffer> buff = std::shared_ptr<capture_memory_buffer>{ new capture_memory_buffer{std::move(val)} };
			return buff;
		};

		copy_memory_buffer::copy_memory_buffer(char* data, size_t size) {
			data_ = std::make_unique<std::vector<char>>();
			data_->resize(size);
			std::memcpy(data_->data(), data, size);
			buffer_ = asio::buffer(*data_);
		}

		std::shared_ptr<memory_buffer> copy_memory_buffer::make_shared(char* data, size_t size) {
			std::shared_ptr<copy_memory_buffer> buff = std::shared_ptr<copy_memory_buffer>{ new copy_memory_buffer{data,size} };
			return buff;
		};

		std::ostream& operator << (std::ostream& os, const  std::shared_ptr<memory_buffer>& val) {
			std::vector<char> vec;
			vec.resize(val->begin()->size());
			std::memcpy(vec.data(), val->begin()->data(), vec.size());
			for (const auto& charelement : vec)
				os << charelement;
			os << "\n";
			return os;
		}
	}
}