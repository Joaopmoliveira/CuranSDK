#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <csignal>
#include <thread>
#include <sstream>
#include <asio.hpp>
#include <cmath>
#include <type_traits>
#include "header_acessor.h"
#include "watchdogmessage.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"

// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

constexpr size_t number_of_display_variables = 3;



asio::io_context io_content;
asio::ip::tcp::socket* socket_pointer = nullptr;

void signal_handler(int val){
    if(socket_pointer!=nullptr)
        socket_pointer->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_content.stop();
}

// variables are global to simplify the call to the interface command
grayscale_image_1 image;
gps_reading gps_read;

void interface(vsg::CommandBuffer& cb){
    ImGui::Begin("Angle Display"); // Create a window called "Hello, world!" and append into it.
    ImGui::BulletText("Move your mouse to change the data!");
    ImGui::BulletText("This example assumes 60 FPS. Higher FPS requires larger buffer size.");
	static std::array<ScrollingBuffer,number_of_display_variables> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < number_of_display_variables ; ++index){
			std::string loc = "accel "+std::to_string(index);
            buffers[index].AddPoint(t,(float)gps_read.acceleration[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
        ImPlot::EndPlot();
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
};

void render_scene(const watchdog_message& message,const std::vector<unsigned char>& shared_memory_copy,std::mutex& shared_memory_acess,std::atomic<bool>& continue_running){
    constexpr grayscale_image_1_layout layout;
    std::vector<unsigned char> shared_memory_blob;
    shared_memory_blob.resize(layout.data_size);
    grayscale_image_1 image;
    image.data = shared_memory_blob.data();
    gps_reading gps_read;
    curan::renderable::ImGUIInterface::Info info_gui{interface};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = true;
    info.screen_number = 0;
    info.title = "myviewer";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{1000, 800};
    info.window_size = size;
    curan::renderable::Window window{info};
    while(continue_running.load()){
        //now that 
        {
            std::lock_guard<std::mutex> g{shared_memory_acess};
            if(message.gps_reading_present)
                copy_from_shared_memory_to_gps_reading(shared_memory_copy.data(),gps_read);
        }
           
        {
            std::lock_guard<std::mutex> g{shared_memory_acess};
             if(message.image_reading_present)
                copy_from_shared_memory_to_grayscale_image_1(shared_memory_copy.data(),image);
        }
        if(!window.run_once())
            return ;
            
    }

};

int main(){
    std::signal(SIGINT,signal_handler);

    constexpr watchdog_message_layout message_layout;
    constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
    std::array<unsigned char,watchdog_message_size> asio_memory_buffer;

    unsigned int port = 50001;
    asio::ip::tcp::endpoint endpoit(asio::ip::tcp::v4(), port);
    asio::ip::tcp::acceptor acceptor(io_content,endpoit);

    asio::error_code ec;
    asio::ip::tcp::socket client_socket = acceptor.accept();
    if (ec){
        std::cout << "failed to run, terminating....\n";
        std::cout << ec.message() << std::endl;
        return 1;
    };
    socket_pointer = &client_socket;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    watchdog_message message;

    constexpr grayscale_image_1_layout layout;
    std::vector<unsigned char> shared_memory_blob;
    shared_memory_blob.resize(layout.data_size);
    std::mutex shared_access;
    std::atomic<bool> continue_running = true;
    auto callable = [&](){
        render_scene(message,shared_memory_blob,shared_access,continue_running);
    };
    std::thread renderer{callable};
    auto shared_memory = SharedMemoryAccessor::create();
   while(!io_content.stopped()){
        asio::read(client_socket, asio::buffer(asio_memory_buffer), asio::transfer_exactly(watchdog_message_size), ec);
        if (ec) {
            std::printf("failed to read information\n terminating....\n");
            io_content.stop();
        }

        {
            std::lock_guard<std::mutex> g{shared_access};
            copy_from_memory_to_watchdog_message(asio_memory_buffer.data(),message);
            std::chrono::time_point currently = std::chrono::time_point_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now()
            );
            std::chrono::duration millis_since_utc_epoch = currently.time_since_epoch();
            message.client_receive_timestamp = millis_since_utc_epoch.count();
        }
        
        {
            std::lock_guard<std::mutex> g{shared_access};
            std::memcpy(shared_memory_blob.data(),shared_memory->get_shared_memory_address(),1);
        }
        
        copy_from_watchdog_message_to_memory(asio_memory_buffer.data(),message);
        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send control action\n terminating....\n");
        }
    }
    continue_running = false;
    renderer.join();
}