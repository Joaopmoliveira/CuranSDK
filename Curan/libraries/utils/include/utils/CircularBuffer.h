#ifndef CURAN_CIRCULAR_BUFFER_HEADER_FILE_
#define CURAN_CIRCULAR_BUFFER_HEADER_FILE_

#include <memory>
#include <optional>
#include <vector>

namespace curan {
namespace utilities {

/*
The CircularBuffer class is a contiguous block of memory, 
where when we push an object of the template type 
it is added to the next memory location. 

When we arrive at the maximum number of elements of the circular buffer 
the next object is placed at the starting index, 
and the same logic applies.

This is useful for memory locality and 
to control the maximum number of elements in a queue. 

Consider the following example. We want a circular buffer with three elements


int main(){
    CircularBuffer<double> buf{3};

% At the beginning, the buffer is just allocated normally. 
Then, we put a value inside our buffer.

    buf.put(1.0);

% Currently the values are [1.0 0.0 0.0]
% Now we can put more values

    buf.put(2.0);
    buf.put(3.0);

% notice that the buffer will now be full
% with the following sequence [1.0 2.0 3.0]
% If we put another value inside then we have

    buf.put(4.0);

% The buffer will now contain the values [4.0 2.0 3.0]

% Another nice feature of the circular buffer is that 
% e can use a range for loop with diminishes the clutter
% of looping through its elements as in 

    std::cout << "the buffer values are:\n"
    for(const auto& val : buf)
        std::printf("%f ",val);

    return 0;
};

*/

template <class T>
class CircularBuffer {

    using LinearView = std::vector<T>;

    enum class allocation{
        LAZY_LINEAR_ALLOCATION,
        PRE_ALLOCATE_LINEAR_ARRAY
    };

public:

	explicit CircularBuffer(size_t size, allocation alloc = allocation::LAZY_LINEAR_ALLOCATION) :
		buf_(size),
		max_size_(size)
	{ switch(alloc){
        case allocation::PRE_ALLOCATE_LINEAR_ARRAY:
            linear_buf_.resize(size);
        break;
        case allocation::LAZY_LINEAR_ALLOCATION:
        default:

        break;
    } }

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
    allocation allocation_strategy;
	std::vector<T> buf_;
    std::vector<T> linear_buf_;
	size_t head_ = 0;
	size_t tail_ = 0;
	const size_t max_size_;
	bool full_ = false;
};

}
}

#endif