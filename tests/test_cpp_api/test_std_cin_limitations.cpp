#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <optional>
#include <memory>

/*
enum ReadingStatus{
    WRITING,
    READING,
    UNTOUCHED
};

struct Message{
    ReadingStatus esp1;
    ReadingStatus esp2;
    ReadingStatus esp3;
};

template<typename T>
class wrapped_list{
    std::list<T> temp;
    std::mutex mut;
public:
    template<typename ...U>
    void emplace_back(){
        std::lock_guard<std::mutex> g{mut};
        temp.emplace_back(std::forward(U...));
    }

    void push_back(const T& t){
        std::lock_guard<std::mutex> g{mut};
        temp.push_back(t);
    }

    bool wait_and_pop(T& t){
        std::lock_guard<std::mutex> g{mut};
        if(temp.size()>0){
            t = temp.back();
            temp.pop_back();
            return true;
        }
        return false;
    }
};

struct WritingPrimitive{

};

struct ReadingPrimitive{
    std::atomic<ReadingStatus> current_status = UNTOUCHED;
    bool should_write = false;
    std::atomic<size_t> f_tick_counter = 0;
    std::mutex mut;
    const size_t f_sensor_multiple;

    ReadingPrimitive(size_t sensor_multiple) : f_sensor_multiple{sensor_multiple}{

    }
    
    operator bool() const{
        return should_write;
    }

    bool will_write() const {
         return !(f_tick_counter+1% f_sensor_multiple);
    }

    void status(ReadingStatus in_status){
        current_status = in_status;
    }

    ReadingStatus status(){
         return current_status;
    }

    void finished(){
        current_status = UNTOUCHED;
    }

    size_t operator++(){
        if(!(f_tick_counter+1% f_sensor_multiple))
            should_write = true;
        return ++f_tick_counter;
    }

};

void esp1(ReadingPrimitive& prim){
    while(true){
        if(prim){ // we have orders to write into the shared memory
            //mimic that we wrote to a shared memory
            prim.finished();
        }

        //query sensor
    }
}

void esp2(ReadingPrimitive& prim){
    while(true){
        if(prim){ // we have orders to write into the shared memory
            //mimic that we wrote to a shared memory
            prim.finished();
        }

        //query sensor
    }
}

void esp3(ReadingPrimitive& prim){
    while(true){
        if(prim){ // we have orders to write into the shared memory
            //mimic that we wrote to a shared memory
            prim.finished();
        }

        //query sensor
    }
}

void sensors(wrapped_list<Message>& pipe){
    ReadingPrimitive prim1{3};
    ReadingPrimitive prim2{10};
    ReadingPrimitive prim3{30};

    std::thread esp1_thread {prim1};
    std::thread esp2_thread {prim2};
    std::thread esp3_thread {prim3};

    Message watchdogmessage;

    while(true){
        // read message from watchdog
        if(!pipe.wait_and_pop(watchdogmessage))
            return;

        // before we tick the ReadingPrimitives we copy the status of acess of the shared memory into the primities
        // but the caveat is that we only copy the current memory acess status if its different from writing, becase if 
        // the previous loop was writing, but in this loop we assycronously we have stopped writing, we don't want to override 
        // that result
        if(watchdogmessage.esp1!=WRITING)
            prim1.status(watchdogmessage.esp1);
        if(watchdogmessage.esp2!=WRITING)
            prim2.status(watchdogmessage.esp2);
        if(watchdogmessage.esp3!=WRITING)
            prim3.status(watchdogmessage.esp3);

        if(prim1.will_write() && !(watchdogmessage.esp1 & UNTOUCHED))
            return; //if we would write in this loop yet the message is still not free due to the controller reading from the shared memory, we failed to respect the total sample time
        else
            prim1.status(WRITING);

        if(prim2.will_write() && !(watchdogmessage.esp2 & UNTOUCHED))
            return; //if we would write in this loop yet the message is still not free due to the controller reading from the shared memory, we failed to respect the total sample time
        else
            prim2.status(WRITING);

        if(prim3.will_write() && !(watchdogmessage.esp3 & UNTOUCHED))
            return; //if we would write in this loop yet the message is still not free due to the controller reading from the shared memory, we failed to respect the total sample time
        else
            prim3.status(WRITING);


        // update all ticks of the reading primitives
        ++prim1, ++prim2, ++prim3;

        watchdogmessage.esp1 = prim1.current_status;
        watchdogmessage.esp2 = prim2.current_status;
        watchdogmessage.esp3 = prim3.current_status;

        // write message to watchdog
        pipe.push_back(watchdogmessage);
    }

    esp1_thread.join();
    esp2_thread.join();
    esp3_thread.join();
}

void watchdog(wrapped_list<Message>& pipe_sensors,wrapped_list<Message>& pipe_controller){
    Message watchdogmessage;

    while(true){
        // read control action from shared memory

        // tick sensor
        pipe_sensors.push_back(watchdogmessage);

        // read sensor ack
        if(pipe_sensors.wait_and_pop(watchdogmessage))
            return; // we failed to receive a message from the 

        // tick controller
        pipe_controller.push_back(watchdogmessage);

        // read controller ack
        if(pipe_controller.wait_and_pop(watchdogmessage))
            return; // we failed to receive a message from the 
    }
}

void controller(wrapped_list<Message>& pipe){
    Message watchdogmessage;



    while(true){
        // read message from watchdog
        if(!pipe.wait_and_pop(watchdogmessage))
            return;


        // write message to watchdog
        pipe.push_back(watchdogmessage);
    }
}

*/
/*

#include "pugixml.hpp"
#include <cassert>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>
#include <filesystem>

enum ErrorCode{
    SUCCESS = 0,
    PATH_NOT_FOUND,
    CHILD_NOT_FOUND,
    ROBOT_DEVICE_NOT_FOUND,
    INVALID_ARGUMENT,
};

[[nodiscard]] ErrorCode modify_robot_time_offset(const std::filesystem::path& path, std::string device_identifier,double new_value){
    bool found_the_desired_device = false;
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());
    if(!result)
        return PATH_NOT_FOUND;
    if(!doc.child("PlusConfiguration").child("DataCollection"))
        return CHILD_NOT_FOUND;
    for(auto val : doc.child("PlusConfiguration").child("DataCollection").children()){
        for(auto atrib : val.attributes()){
            if(!std::string{atrib.name()}.compare("LocalTimeOffsetSec") && !std::string{val.attribute("Id").value()}.compare(device_identifier)){
                std::string value_before{atrib.value()};
                double contained_value = 0.0;
                auto [ptr, ec] = std::from_chars(value_before.data(), value_before.data() + value_before.size(), contained_value);
 
                if (ec == std::errc())
                    found_the_desired_device = true;
                else if (ec == std::errc::invalid_argument)
                    return INVALID_ARGUMENT;
                else if (ec == std::errc::result_out_of_range)
                    return INVALID_ARGUMENT;
                if(found_the_desired_device){
                    auto strin = std::to_string(contained_value+new_value);
                    atrib.set_value(strin.data());
                    break;
                }

            }
        }
        if(found_the_desired_device)
            break;
    }
    return (found_the_desired_device) ? SUCCESS : ROBOT_DEVICE_NOT_FOUND;
}

int main(){
    const std::string device_id{"ROBOT"};
    switch(modify_robot_time_offset("C:/Dev/Curan/resources/plus_config/plus_spacial_calib_robot_xml/robot_image.xml",device_id,10)){
        case ErrorCode::SUCCESS:
            std::cout << "success" << std::endl;
        break;
        case ErrorCode::CHILD_NOT_FOUND:
            std::cout << "data collection not found" << std::endl;
        break;
        case ErrorCode::INVALID_ARGUMENT:
            std::cout << "invalid argument" << std::endl; 
        break;
        case ErrorCode::PATH_NOT_FOUND:
            std::cout << "path not found" << std::endl;
        break;
        case ErrorCode::ROBOT_DEVICE_NOT_FOUND:
            std::cout << "device id: " << device_id << " - device not found" << std::endl;
        break;
        default:
            std::cout << "unknown error" << std::endl;
    }
    return 0;
}
*/

#include <functional>
#include <type_traits>

template<typename, typename = void>
constexpr bool is_type_complete_v = false;

template<typename T>
constexpr bool is_type_complete_v
    <T, std::void_t<decltype(sizeof(T))>> = true;

template<typename T>
struct Client{
    static_assert(std::is_invocable_v<decltype(T::start),std::shared_ptr<Client<T>>>, "the protocol must have a static start() function that receives a templated client");
    static_assert(is_type_complete_v<typename T::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");
    
    Client(){

    }
};

template<typename T>
struct Server{
    static_assert(std::is_invocable_v<decltype(T::start),std::shared_ptr<Client<T>>>, "the protocol must have a static start() function that receives a templated client");
    static_assert(is_type_complete_v<typename T::signature>, "the protocol must have signature type function that broadcasts the the protocol messages");

    std::list<Client<T>> clients;
    std::list<typename T::signature> list_of_callbacks;
    Server(){

    }
};

class GoodProtocol{
    public:
	using signature = std::function<void(const size_t&, const std::error_code&, std::string_view value)>;
			
	static void start(std::shared_ptr<Client<GoodProtocol>> client){
			
	};
};



class PretendToBeGoodProtocol{
    public:	
    using signature = std::function<void(const size_t&, const std::error_code&, std::string_view value)>;
	void start(std::shared_ptr<Client<PretendToBeGoodProtocol>> client){
			
	};
};

class BadProtocol{
    public:
	using signature = std::function<void(const size_t&, const std::error_code&, std::string_view value)>;
			
	static void start(){
			
	};
};

class BadProtocolForLackOfSignature{
    public:
			
	static void start(std::shared_ptr<Client<BadProtocolForLackOfSignature>> client){
			
	};
};

#include <array>

std::array<double,3> arr;

std::array<double,3> arr2;

void incrementor(size_t adress){

    std::cout << adress << std::endl;
    arr2[adress] = 1;
}

int main(){
    size_t t = 0;
    arr[t++] = 1;
    std::cout << t << std::endl;
    for(const auto& v : arr)
        std::cout << v << " ";
    std::cout << std::endl;
    t = 0;
    incrementor(t++);
    std::cout << t << std::endl;
    for(const auto& v : arr2)
        std::cout << v << " ";
    std::cout << std::endl;
    //Server<PretendToBeGoodProtocol> protocol1;

    //Server<BadProtocol> protocol2;

    //Server<BadProtocolForLackOfSignature> protocol3;
    return 0;
}