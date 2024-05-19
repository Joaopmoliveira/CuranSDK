#include "asio.hpp"
#include <any>
#include <vector>
#include <memory>
#include <string>
#include <iostream>

class MemoryBuffer
{
public:
    typedef asio::const_buffer value_type;
    typedef const asio::const_buffer *const_iterator;

    virtual const asio::const_buffer *begin() const = 0;
    virtual const asio::const_buffer *end() const = 0;

    friend std::ostream &operator<<(std::ostream &os, const std::shared_ptr<MemoryBuffer> &val);
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

std::shared_ptr<MemoryBuffer> foo(){
    std::vector<char> loc = {'y','e','s'};
    std::shared_ptr<std::vector<char>> buffer_1 = std::make_shared<std::vector<char>>(std::move(loc));
    auto capture_1 = CaptureBuffer::make_shared(buffer_1->data(),buffer_1->size(),buffer_1);
    std::cout << "first buffer contains: " << capture_1 << std::endl;
    return capture_1;
}

std::shared_ptr<MemoryBuffer> bar(){
    std::shared_ptr<std::string> buffer_2 = std::make_shared<std::string>("not");
    auto capture_2 = CaptureBuffer::make_shared(buffer_2->data(),buffer_2->size(),buffer_2); 
    std::cout << "second buffer contains: " << capture_2 << std::endl;
    return capture_2;
}

int main(){
    auto capture_1 = foo();
    auto capture_2 = bar();
    std::cout << "first buffer contains: " << capture_1 << std::endl;
    std::cout << "second buffer contains: " << capture_2 << std::endl;
    return 0;
}