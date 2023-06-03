#include "utils/MemoryUtils.h"

namespace curan {
namespace utilities {

CaptureBuffer::CaptureBuffer(binding&& val) : val_(val) {
buffer_ = val();
}

std::shared_ptr<MemoryBuffer> CaptureBuffer::make_shared(binding&& val) {
	std::shared_ptr<CaptureBuffer> buff = std::shared_ptr<CaptureBuffer>{ new CaptureBuffer{std::move(val)} };
	return buff;
};

CopyBuffer::CopyBuffer(char* data, size_t size) {
	data_ = std::make_unique<std::vector<char>>();
	data_->resize(size);
	std::memcpy(data_->data(), data, size);
	buffer_ = asio::buffer(*data_);
}

std::shared_ptr<MemoryBuffer> CopyBuffer::make_shared(char* data, size_t size) {
	std::shared_ptr<CopyBuffer> buff = std::shared_ptr<CopyBuffer>{ new CopyBuffer{data,size} };
	return buff;
};

std::ostream& operator << (std::ostream& os, const  std::shared_ptr<MemoryBuffer>& val) {
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