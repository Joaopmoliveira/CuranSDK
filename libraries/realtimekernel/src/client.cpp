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
#include "rendering/Mesh.h"
#include "rendering/DynamicHeight.h"


constexpr size_t number_of_display_variables = 3;

asio::io_context io_content;
asio::ip::tcp::socket* socket_pointer = nullptr;

void signal_handler(int val){
    if(socket_pointer!=nullptr)
        socket_pointer->shutdown(asio::ip::tcp::socket::shutdown_both);
    io_content.stop();
}

int counter;
	double latitude;
	double longitude;
	double height;
	double velocity[3];
	double acceleration[3];
	double gforce;
	double orientation[3];
	double angular_velocity[3];
	double standard_deviation[3];

struct Params{
    bool showAcceleration = false; // you can toggle this with your own EventHandler and key
    bool showTiming = false;
    bool showGlobalCoordinates = false;
    bool showVelocity = false;
    bool showOrientation = true;
    bool showAngularVelocity = false;
    bool showStandardDeviation = false;
} parameters;

// variables are global to simplify the call to the interface command
grayscale_image_1 image;
gps_reading gps_read;
watchdog_message global_message;

void interface(vsg::CommandBuffer& cb){
    ImGui::Begin("General Information"); // Create a window called "Hello, world!" and append into it.
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;

    constexpr ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
    
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");
    ImGui::Checkbox("Show Acceleration", &parameters.showAcceleration); // Edit bools storing our window open/close state
    ImGui::Checkbox("Show Sample Times", &parameters.showTiming);
    ImGui::Checkbox("Show World Coordinates", &parameters.showGlobalCoordinates);
    ImGui::Checkbox("Show Velocity", &parameters.showVelocity);
    ImGui::Checkbox("Show Orientation", &parameters.showOrientation);
    ImGui::Checkbox("Show Angular Velocities", &parameters.showAngularVelocity);
    ImGui::Checkbox("Show Stander Deviations", &parameters.showStandardDeviation);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();

	static std::array<curan::renderable::ScrollingBuffer,number_of_display_variables> acceleration_buffers;
    static std::array<curan::renderable::ScrollingBuffer,number_of_display_variables> standard_deviation_buffers;
    static std::array<curan::renderable::ScrollingBuffer,number_of_display_variables> velocity_buffers;
    static std::array<curan::renderable::ScrollingBuffer,number_of_display_variables> orientation_buffers;
    static std::array<curan::renderable::ScrollingBuffer,number_of_display_variables> angular_velocity_buffers;

	for(size_t index = 0; index < number_of_display_variables ; ++index){
        acceleration_buffers[index].AddPoint(t,(float)gps_read.acceleration[index]);
        velocity_buffers[index].AddPoint(t,(float)gps_read.velocity[index]);
        orientation_buffers[index].AddPoint(t,(float)gps_read.orientation[index]);
        angular_velocity_buffers[index].AddPoint(t,(float)gps_read.angular_velocity[index]);
        standard_deviation_buffers[index].AddPoint(t,(float)gps_read.standard_deviation[index]);
    }
        
    if(parameters.showAcceleration){
        ImGui::Begin("Acceleration"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##ACCEL", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    for(size_t index = 0; index < number_of_display_variables ; ++index){
			    std::string loc = "accel "+std::to_string(index);
			    ImPlot::PlotLine(loc.data(), &acceleration_buffers[index].Data[0].x, &acceleration_buffers[index].Data[0].y, acceleration_buffers[index].Data.size(), 0, acceleration_buffers[index].Offset, 2 * sizeof(float));
		    }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

	static curan::renderable::ScrollingBuffer longitude_buffer;
    static curan::renderable::ScrollingBuffer latitude_buffer;
    static curan::renderable::ScrollingBuffer height_buffer;

    longitude_buffer.AddPoint(t,(float)gps_read.longitude);
    latitude_buffer.AddPoint(t,(float)gps_read.latitude);
    height_buffer.AddPoint(t,(float)gps_read.height);

    if(parameters.showGlobalCoordinates){
        ImGui::Begin("Coordinates"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##COORD", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
            ImPlot::PlotLine("longitute", &longitude_buffer.Data[0].x, &longitude_buffer.Data[0].y, longitude_buffer.Data.size(), 0, longitude_buffer.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("latitude", &latitude_buffer.Data[0].x, &latitude_buffer.Data[0].y, latitude_buffer.Data.size(), 0, latitude_buffer.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("height", &height_buffer.Data[0].x, &height_buffer.Data[0].y, height_buffer.Data.size(), 0, height_buffer.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    if(parameters.showVelocity){
        ImGui::Begin("Velocity"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##VELOCITY", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    for(size_t index = 0; index < number_of_display_variables ; ++index){
			    std::string loc = "vel "+std::to_string(index);
			    ImPlot::PlotLine(loc.data(), &velocity_buffers[index].Data[0].x, &velocity_buffers[index].Data[0].y, velocity_buffers[index].Data.size(), 0, velocity_buffers[index].Offset, 2 * sizeof(float));
		    }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    if(parameters.showOrientation){
        ImGui::Begin("Orientation"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##ORIENT", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    for(size_t index = 0; index < number_of_display_variables ; ++index){
			    std::string loc = "angl "+std::to_string(index);
			    ImPlot::PlotLine(loc.data(), &orientation_buffers[index].Data[0].x, &orientation_buffers[index].Data[0].y, orientation_buffers[index].Data.size(), 0, orientation_buffers[index].Offset, 2 * sizeof(float));
		    }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    if(parameters.showAngularVelocity){
        ImGui::Begin("Angular Velocity"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##ANGVEL", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    for(size_t index = 0; index < number_of_display_variables ; ++index){
			    std::string loc = "angl vel "+std::to_string(index);
			    ImPlot::PlotLine(loc.data(), &angular_velocity_buffers[index].Data[0].x, &angular_velocity_buffers[index].Data[0].y, angular_velocity_buffers[index].Data.size(), 0, angular_velocity_buffers[index].Offset, 2 * sizeof(float));
		    }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    if(parameters.showStandardDeviation){
        ImGui::Begin("Uncertanty"); // Create a window called "Hello, world!" and append into it.
        if (ImPlot::BeginPlot("##DEVIATION", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    for(size_t index = 0; index < number_of_display_variables ; ++index){
			    std::string loc = "var "+std::to_string(index);
			    ImPlot::PlotLine(loc.data(), &standard_deviation_buffers[index].Data[0].x, &standard_deviation_buffers[index].Data[0].y, standard_deviation_buffers[index].Data.size(), 0, standard_deviation_buffers[index].Offset, 2 * sizeof(float));
		    }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

    if(parameters.showTiming){
        ImGui::Begin("Sample Times"); // Create a window called "Hello, world!" and append into it.
	    static curan::renderable::ScrollingBuffer sensors_receive_timestamp;
        static curan::renderable::ScrollingBuffer sensors_send_timestamp;
        static curan::renderable::ScrollingBuffer watchdog_sensor_receive_timestamp;
        static curan::renderable::ScrollingBuffer watchdog_client_reqst_timestamp;
        static curan::renderable::ScrollingBuffer client_receive_timestamp;

        sensors_receive_timestamp.AddPoint(t,(double)1e-3*(global_message.sensors_receive_timestamp-global_message.watchdog_sensor_reqst_timestamp));
        sensors_send_timestamp.AddPoint(t,(double)1e-3*(global_message.sensors_send_timestamp-global_message.watchdog_sensor_reqst_timestamp));
        watchdog_sensor_receive_timestamp.AddPoint(t,(double)1e-3*(global_message.watchdog_sensor_receive_timestamp-global_message.watchdog_sensor_reqst_timestamp));
        watchdog_client_reqst_timestamp.AddPoint(t,(double)1e-3*(global_message.watchdog_client_reqst_timestamp-global_message.watchdog_sensor_reqst_timestamp));
        client_receive_timestamp.AddPoint(t,(double)1e-3*(global_message.client_receive_timestamp-global_message.watchdog_sensor_reqst_timestamp));

        if (ImPlot::BeginPlot("##SAMPLETIMES", ImVec2(-1,150))) {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		    ImPlot::PlotLine("S_rec", &sensors_receive_timestamp.Data[0].x, &sensors_receive_timestamp.Data[0].y, sensors_receive_timestamp.Data.size(), 0, sensors_receive_timestamp.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("S_sed",  &sensors_send_timestamp.Data[0].x, &sensors_send_timestamp.Data[0].y, sensors_send_timestamp.Data.size(), 0, sensors_send_timestamp.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("W_rec",  &watchdog_sensor_receive_timestamp.Data[0].x, &watchdog_sensor_receive_timestamp.Data[0].y, watchdog_sensor_receive_timestamp.Data.size(), 0, watchdog_sensor_receive_timestamp.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("W_cli",  &watchdog_client_reqst_timestamp.Data[0].x, &watchdog_client_reqst_timestamp.Data[0].y, watchdog_client_reqst_timestamp.Data.size(), 0, watchdog_client_reqst_timestamp.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("C_rec",  &client_receive_timestamp.Data[0].x, &client_receive_timestamp.Data[0].y, client_receive_timestamp.Data.size(), 0, client_receive_timestamp.Offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
        ImGui::End();
    }
};

void render_scene(const watchdog_message& message,const std::vector<unsigned char>& shared_memory_copy,std::mutex& shared_memory_acess){
    std::vector<unsigned char> shared_memory_blob;
    shared_memory_blob.resize(shared_memory_copy.size());
    image.data = shared_memory_blob.data();
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


    std::filesystem::path swingcar_path = CURAN_COPIED_RESOURCE_PATH"/swing/R4F_stable.glb";
    curan::renderable::Mesh::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.mesh_path = swingcar_path;
    vsg::ref_ptr<curan::renderable::Renderable> swingcar = curan::renderable::Mesh::make(create_info);
    window << swingcar;

    curan::renderable::DynamicHeight::Info infotexture;
    infotexture.height = 100;
    infotexture.width = 100;
    infotexture.depth = 100;
    infotexture.origin = {0.0,0.0,0.0};
    infotexture.spacing = {0.005,0.005,0.005};
    infotexture.builder = vsg::Builder::create();
    auto dynamic_texture = curan::renderable::DynamicHeight::make(infotexture);
    dynamic_texture->update_transform(vsg::rotate(vsg::radians(90.0),0.0,1.0,0.0)*vsg::translate(-1.0,-0.5,-2.0));
    swingcar->append(dynamic_texture);

    std::atomic<bool> continue_updating_heightfield = true;
    auto callable = [&](){
        float value = 1.0;
        auto updateBaseTexture = [&value](vsg::floatArray2D& image)
        {
            using value_type = typename vsg::floatArray2D::value_type;
            for (int r = 0; r < image.height(); ++r)
            {
                float r_ratio = static_cast<float>(r) / static_cast<float>(image.height() - 1);
                value_type* ptr = &image.at(0, r);
                for (int c = 0; c < image.width(); ++c)
                {
                    float c_ratio = static_cast<float>(c) / static_cast<float>(image.width() - 1);

                    vsg::vec2 delta((r_ratio - 0.5f), (c_ratio - 0.5f));

                    float angle = atan2(delta.x, delta.y);

                    float distance_from_center = vsg::length(delta);

                    float intensity = 0.1*(sin(1.0 * angle + 30.0f * distance_from_center + 10.0f * value) + 1.0f) * 0.5f;
                    *ptr = intensity;
                    ++ptr;
                }
            }
        };
        while(continue_updating_heightfield){
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            dynamic_texture->cast<curan::renderable::DynamicHeight>()->update_texture(updateBaseTexture);
            value += 0.01f;
        }
    };
    std::thread updater{callable};

    while(window.run_once()){
        {
            std::lock_guard<std::mutex> g{shared_memory_acess};
            global_message = message;
            if(message.gps_reading_present)
                copy_from_shared_memory_to_gps_reading(shared_memory_copy.data(),gps_read);
        }

        //swingcar->cast<curan::renderable::Mesh>()->update_transform(vsg::translate(gps_read.latitude,gps_read.longitude,gps_read.height));
           
        {
            std::lock_guard<std::mutex> g{shared_memory_acess};
             if(message.image_reading_present)
                copy_from_shared_memory_to_grayscale_image_1(shared_memory_copy.data(),image);
        }     
    }
    continue_updating_heightfield = false;
    updater.join();
    std::raise(SIGINT);
};

int main(){
    std::signal(SIGINT,signal_handler);

    constexpr watchdog_message_layout message_layout;
    constexpr size_t watchdog_message_size = message_layout.image_reading_present_address+message_layout.image_reading_present_size;
    std::array<unsigned char,watchdog_message_size> asio_memory_buffer;

    unsigned int port = 50010;
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
    watchdog_message message;
    auto shared_memory = SharedMemoryAccessor::create();
    constexpr grayscale_image_1_layout layout;
    std::vector<unsigned char> shared_memory_blob;
    shared_memory_blob.resize(shared_memory->size());
    std::mutex shared_access;
    auto callable = [&](){
        render_scene(message,shared_memory_blob,shared_access);
    };
    std::thread renderer{callable};

    if(shared_memory->get_shared_memory_address()==nullptr){
        std::cout << "failure to create shared memory\n";
        return 1;
    }
   while(!io_content.stopped()){
        asio::read(client_socket, asio::buffer(asio_memory_buffer), asio::transfer_exactly(watchdog_message_size), ec);
        if (ec) {
            std::printf("failed to read information\n terminating....\n");
            io_content.stop();
        }

        {
            std::lock_guard<std::mutex> g{shared_access};
            copy_from_memory_to_watchdog_message(asio_memory_buffer.data(),message);
            std::chrono::time_point currently = std::chrono::time_point_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now()
            );
            std::chrono::duration millis_since_utc_epoch = currently.time_since_epoch();
            message.client_receive_timestamp = millis_since_utc_epoch.count();
        }

        {
            std::lock_guard<std::mutex> g{shared_access};
            std::memcpy(shared_memory_blob.data(),shared_memory->get_shared_memory_address(),shared_memory->size());
        }
        
        copy_from_watchdog_message_to_memory(asio_memory_buffer.data(),message);
        asio::write(client_socket,asio::buffer(asio_memory_buffer),asio::transfer_exactly(watchdog_message_size),ec);
        if(ec){
            std::printf("failed to send control action\n terminating....\n");
            io_content.stop();
        }
    }
    renderer.join();
}