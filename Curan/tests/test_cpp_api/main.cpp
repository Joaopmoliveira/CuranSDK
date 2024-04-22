#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include <thread>

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

enum class Peripheral{
    GPS_READING = 0,
    CAMERA = 1,
    COUNT = 2
};

struct Sincronizer{
    std::atomic<bool> valid = false;
    std::array<std::atomic<bool>,static_cast<int>(Peripheral::COUNT)> flags;
    std::mutex mut;
    std::condition_variable cv;
    size_t written = 0;

    template<Peripheral index>
    inline bool should_read(){
        static_assert(to_underlying(index)<static_cast<int>(Peripheral::COUNT),"the maximum index to read must be smaller or equal than the number of peripherals");
        if(flags[to_underlying(index)]){
            flags[to_underlying(index)] = false;
            return true;
        }
        return false;
    };

    template<typename... Ts>
    void read(Ts... args){
        constexpr size_t number_of_args = sizeof...(Ts);
        internal_read(args...);
        static_assert(sizeof... (Ts)<=static_cast<int>(Peripheral::COUNT), "the number of requested reads is larger than the number of peripherals");
        wait<number_of_args>();
    }

    inline void wrote(){
        std::lock_guard<std::mutex> g{mut};
        ++written;
        cv.notify_one();
    };

    inline void stop(){
        valid.store(true,std::memory_order_relaxed);
    };

    inline bool is_stoped(){
        return valid.load(std::memory_order_relaxed);
    };

private:
    void internal_read(Peripheral last){
        flags[to_underlying(last)] = true;
    }

    template<typename... Ts>
    void internal_read(Peripheral index,Ts... ts){
        flags[to_underlying(index)] = true;
        internal_read(ts...);
    };

    template<size_t number_of_args>
    inline void wait(){
        std::unique_lock<std::mutex> lk{mut};
        cv.wait(lk,[this](){return written == number_of_args;});
        written = 0;
    };

};

void camera_reader(Sincronizer& sincronizer){
    while(!sincronizer.is_stoped()){
        if(sincronizer.should_read<Peripheral::CAMERA>()){
            std::cout << "written camera\n";
            sincronizer.wrote();
        }
    }
}

void gps_reader(Sincronizer& sincronizer){
    while(!sincronizer.is_stoped()){
        if(sincronizer.should_read<Peripheral::GPS_READING>()){
            std::cout << "written gps\n";
            sincronizer.wrote();
        }
    }
}

Sincronizer sincronizer;
void signal_handler(int val)
{
    sincronizer.stop();
}

template<typename T>
class unique_ptr_mine{
    T* my_pointer = nullptr;
public:
    unique_ptr_mine(T* other) : my_pointer{other}{

    }

    unique_ptr_mine(const unique_ptr_mine& other) = delete;
    void operator= (const unique_ptr_mine& other) = delete;

    unique_ptr_mine(unique_ptr_mine&& other) : my_pointer{other.get()}{
        other.my_pointer = nullptr;
    }

    void operator= (unique_ptr_mine&& other){
        my_pointer = other.get();
        other.my_pointer = nullptr;
    }

    ~unique_ptr_mine(){
        if(my_pointer!=nullptr)
            delete my_pointer;
    }

    T* get(){
        return my_pointer;
    }


};

/*
    unique_ptr_mine //g g->0012341423
    unique_ptr_mine //v g->nullptr v->0012341423
    ~unique_ptr_mine //v
    unique_ptr_mine //g g->0012341423

*/

unique_ptr_mine<double> gg{nullptr};

void fn(unique_ptr_mine<double>&& v){
    double* a = v.get();
    *a = 4.0;
    gg = std::move(v);
}

int main2(){
    unique_ptr_mine<double> g{new double{0}};
    double* f = g.get();
    fn(std::move(g));
    return 0;
}


int main(){
    volatile double bb = 10.0;
    
    std::thread reader{[&](){ bool value = true;  while(true){bb = (value) ? 10.0 : 5.0e4; value = !value;} }};
    std::thread writer{[&](){ while(true) std::cout << bb << "\n";}};
    reader.join();
    writer.join();
    return 0;
}


int main1(){
    std::signal(SIGINT,signal_handler);
    std::thread camera_thread{[&](){camera_reader(sincronizer);}};
    std::thread gps_thread{[&](){gps_reader(sincronizer);}};
    for(size_t counter = 0;!sincronizer.is_stoped(); ++counter){
        if(counter % 5 == 0){
            sincronizer.read(Peripheral::CAMERA,Peripheral::GPS_READING);
            continue;
        }
        sincronizer.read(Peripheral::GPS_READING);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    camera_thread.join();
    gps_thread.join();
}