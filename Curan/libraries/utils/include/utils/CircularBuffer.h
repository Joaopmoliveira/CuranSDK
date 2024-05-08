#ifndef CURAN_CIRCULAR_BUFFER_HEADER_FILE_
#define CURAN_CIRCULAR_BUFFER_HEADER_FILE_

#include <memory>
#include <optional>
#include <vector>
#include <algorithm>

namespace curan {
namespace utilities {

/*
The circular buffer class is a contiguous block of memory, 
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

	explicit CircularBuffer(size_t size, allocation alloc = allocation::PRE_ALLOCATE_LINEAR_ARRAY) :
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

    T head(){
        return buf_[ (head_-1) % max_size_];
    }

    T tail(){
        return buf_[tail_];
    }

    auto begin() { return buf_.begin(); }
    auto end() { return buf_.end(); }

    template<class UnaryOp>
    void operate(UnaryOp opt){
        std::for_each(buf_.begin(),buf_.end(),opt);
    }

    template<class UnaryOp>
    auto linear_view(UnaryOp opt){
        if(full_){   
            linear_buf_.resize(buf_.size()); 
            std::copy(buf_.begin()+tail_,buf_.begin()+buf_.size(),linear_buf_.begin());
            std::copy(buf_.begin(),buf_.begin()+tail_,linear_buf_.begin()+buf_.size()-tail_);
            std::for_each(linear_buf_.begin(),linear_buf_.end(),opt);
            return linear_buf_;
        }
        linear_buf_.resize(head_); 
        std::copy(buf_.begin(),buf_.begin()+head_,linear_buf_.begin());
        std::for_each(linear_buf_.begin(),linear_buf_.end(),opt);
        return linear_buf_;
    }

    auto linear_view(){
        if(full_){
            linear_buf_.resize(buf_.size()); 
            std::copy(buf_.begin()+tail_,buf_.begin()+buf_.size(),linear_buf_.begin());
            std::copy(buf_.begin(),buf_.begin()+tail_,linear_buf_.begin()+buf_.size()-tail_);
            return linear_buf_;
        }
        linear_buf_.resize(head_);
        std::copy(buf_.begin(),buf_.begin()+head_,linear_buf_.begin());
        return linear_buf_;
    }

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

    const T* data() {
        return buf_.data();
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