#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>
#include "utils/TheadPool.h"
#include "imgui_stdlib.h"

enum WindowSpecification{
    ROI_SPECIFICATION,
    VOLUME_RECONSTRUCTION
};

struct ApplicationState{
    WindowSpecification specification{ROI_SPECIFICATION};
    bool operation_in_progress = false;
    bool show_error = false;
    bool show_sucess = false;
    std::string filename;
    std::mutex mut;
    std::string operation_description;
    std::string success_description;
    ImVec2 padding{0, 40};
    std::shared_ptr<curan::utilities::ThreadPool> pool;
    
    ApplicationState(){
        pool = curan::utilities::ThreadPool::create(5);
    }

void showMainWindow(){
    ImGui::Begin("Volume Reconstruction",NULL,ImGuiWindowFlags_MenuBar);
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Mode"))
        {
            if (ImGui::MenuItem("ROI specification", "Ctrl+O")) { 
                if(operation_in_progress && specification != ROI_SPECIFICATION)
                {
                    std::lock_guard<std::mutex> g{mut};
                    show_error = true;
                }
                else
                    specification = ROI_SPECIFICATION; 
            }
            if (ImGui::MenuItem("Reconstruction", "Ctrl+S"))   { 
                if(operation_in_progress && specification != VOLUME_RECONSTRUCTION)
                {
                    std::lock_guard<std::mutex> g{mut};
                    show_error = true;
                }
                else
                    specification = VOLUME_RECONSTRUCTION; 
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    ImGui::Dummy(padding); ImGui::SameLine();
    bool local_copy;
    {
        std::lock_guard<std::mutex> g{mut};
        local_copy = operation_in_progress;
    }
    if(local_copy)
        ImGui::ProgressBar(-1.0f *  ImGui::GetTime());
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::TextWrapped("You can select a region of interest and then inject B-Scans into the volume. You always need to specify the bounding box before reconstruction");                 // Display some text (you can use a format strings too)
    ImGui::Dummy(padding); ImGui::SameLine();
    
    switch(specification){
        case ROI_SPECIFICATION:
            showRegionOfInterestWindow();
        break;
        case VOLUME_RECONSTRUCTION:
        default:
            showReconstructionWindow();
        break;
    }
    
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
    showOverlayErrorWindow();
    showOverlaySuccessWindow();
}

void showReconstructionWindow(){
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::InputText("Filename",&filename);
    if (ImGui::Button("Save Volume")){
        {
            std::lock_guard<std::mutex> g{mut};
            operation_in_progress = true;
            operation_description = "saving volumetric reconstruction";
        }
        pool->submit(curan::utilities::Job{"testing",[this](){
            std::this_thread::sleep_for(std::chrono::seconds(2));
            {
                std::lock_guard<std::mutex> g{mut};
                success_description = "saved volume information";
                operation_in_progress = false;
                show_sucess = true;
            }
        }});
    }
    ImGui::Dummy(padding); ImGui::SameLine();
    if (ImGui::Button("Clear Volume")){
        
    }
};

void showRegionOfInterestWindow(){
    static bool local_record_data = false;

    ImGui::Checkbox("Start Frame Collection", &local_record_data); 
    if(local_record_data) {
        std::lock_guard<std::mutex> g{mut};
        operation_in_progress = true;
        operation_description = "collecting data from ultrasound for region of interest";
    } 
    ImGui::Dummy(padding); ImGui::SameLine();
    if (ImGui::Button("Save Region of Interest")){
        local_record_data = false;
        operation_in_progress = true;
        operation_description = "saving region of interest";
        pool->submit(curan::utilities::Job{"testing",[this](){
            std::this_thread::sleep_for(std::chrono::seconds(2));
            success_description = "saved volume information";
            operation_in_progress = false;
            show_sucess = true;
        }});
    }
};

void showOverlayErrorWindow(){
    if(!show_error)
        return;
    ImGui::Begin("Error Reporting",NULL,ImGuiWindowFlags_MenuBar);
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::TextColored(ImVec4(1,0,0,1),"Operation already in progress...");
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::TextColored(ImVec4(1,0,0,1),operation_description.data());
    ImGui::Dummy(padding); ImGui::SameLine();
    if(ImGui::Button("Ok!")){
        show_error = false;
    }
    ImGui::End();
};

void showOverlaySuccessWindow(){
    if(!show_sucess)
        return;
    ImGui::Begin("Operation Sucessefull",NULL,ImGuiWindowFlags_MenuBar);
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::TextColored(ImVec4(0,1,0,1),"Operation finished");
    ImGui::Dummy(padding); ImGui::SameLine();
    ImGui::TextColored(ImVec4(0,1,0,1),success_description.data());
    ImGui::Dummy(padding); ImGui::SameLine();
    if(ImGui::Button("Ok!")){
        show_sucess = false;
    }
    ImGui::End();
};

};

void interface(vsg::CommandBuffer& cb){
    static ApplicationState application_state;
    application_state.showMainWindow();
}

int main(int argc, char **argv) {
    try {
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
                
        window.run();


    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
