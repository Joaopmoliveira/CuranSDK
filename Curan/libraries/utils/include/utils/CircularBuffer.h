#ifndef CURAN_CIRCULAR_BUFFER_HEADER_FILE_
#define CURAN_CIRCULAR_BUFFER_HEADER_FILE_

#include <memory>
#include <optional>
#include <vector>

namespace curan {
namespace utilities {

template <class T>
class CircularBuffer {

public:

	explicit CircularBuffer(size_t size) :
		buf_(size),
		max_size_(size)
	{  }

	void put(T&& t){
        buf_[head_] = std::move(t);
        tail_ = (full_) ? (tail_ + 1) % max_size_ : tail_;
        head_ = (head_ + 1) % max_size_;
        full_ = head_ == tail_;
    }

	std::optional<T> get(){
        if(full_)
            return std::nullopt;
        auto old_tail = tail_;
	    full_ = false;
	    tail_ = (tail_ + 1) % max_size_;
        return buf_[old_tail];
    }   

    auto begin() { return buf_.begin(); }
    auto end() { return buf_.end(); }

	void reset(){
        head_ = tail_;
	    full_ = false;
    }

	bool empty() const{
        return (!full_ && (head_ == tail_));
    }

	bool full() const{
        return full_;
    }

	size_t capacity() const{
        return max_size_;
    }

	size_t size() const{
        if(full_)
            return max_size_;
	    return (head_ >= tail_) ? head_ - tail_ :  max_size_ + head_ - tail_ ;
    }

private:
	std::vector<T> buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = false;
};

template<typename T>
class LinearBuffer{

    explicit LinearBuffer(const CircularBuffer<T>& ref ) : buf_(ref.size()){}

    auto begin() { return buf_.begin(); }
    auto end() { return buf_.end(); }

    private:
    std::vector<T> buf_;
};

}
}

#endif