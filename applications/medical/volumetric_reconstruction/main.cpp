#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/ImGUIInterface.h"
#include <iostream>
#include "utils/TheadPool.h"
#include "utils/Reader.h"
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include "rendering/Box.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "imageprocessing/BoundingBox4Reconstruction.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "imgui_stdlib.h"
#include <map>
#include <string>
#include "utils/FileStructures.h"

/*
This executable requires:

1. Temporal calibration has been performed on the robot
which translates into a file called temporal_calibration.json
with the date at which the calibration was performed

2. Spatial calibration has been performed on the robot
which translates into a file called spatial_calibration.json
with the date at which the calibration was performed

And it outputs

1. 

*/

using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<OutputPixelType, 3>;

auto return_current_time_and_date = []()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    auto val = std::chrono::system_clock::now();
    ss << val.time_since_epoch().count();
    return ss.str();
};

class RobotState{
    std::atomic<bool> commit_senpuko = false;
    std::atomic<bool> f_record_frame = false;
    std::atomic<bool> f_regenerate_integrated_reconstructor_frame = false;
    std::atomic<bool> f_inject_frame = false;

public:
    vsg::ref_ptr<curan::renderable::Renderable> robot;
    curan::renderable::Window &window_pointer;
    curan::image::BoundingBox4Reconstruction box_class;
    vsg::ref_ptr<curan::renderable::Renderable> rendered_box;
    curan::image::IntegratedReconstructor::Info integrated_volume_create_info;
    vsg::ref_ptr<curan::renderable::Renderable> integrated_volume;
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> dynamic_texture;
    vsg::dmat4 calibration_matrix;
    

    RobotState(curan::renderable::Window &wind) : window_pointer{wind},integrated_volume_create_info{{{0.1,0.1,0.1}}, {{0,0,0}}, {{10,10,10}}, {{{1,0,0},{0,1,0},{0,0,1}}}}
    {
    }

    RobotState(const RobotState &) = delete;
    RobotState(RobotState &&) = delete;
    RobotState &operator=(const RobotState &) = delete;
    RobotState &operator=(RobotState &&) = delete;

    operator bool()
    {
        return commit_senpuko.load();
    }

    void operator()(bool value)
    {
        commit_senpuko.store(value);
    }

    enum RecordStatus
    {
        START_RECORDING_FOR_BOX_UPDATE,
        NOT_RECORDING
    };

    void record_frames(RecordStatus status)
    {
        f_record_frame.store(status == START_RECORDING_FOR_BOX_UPDATE);
    }

    bool record_frames()
    {
        return f_record_frame.load();
    }

    enum GenerateStatus
    {
        GENERATE_VOLUME,
        ALREADY_GENERATED
    };

    void generate_volume(GenerateStatus status)
    {
        //std::cout << "flag \"generate_volume\" updated!!!";
        f_regenerate_integrated_reconstructor_frame.store(status == GENERATE_VOLUME);
    }

    bool generate_volume()
    {
        return f_regenerate_integrated_reconstructor_frame.load();
    }

    enum InjectVolumeStatus
    {
        INJECT_FRAME,
        FREEZE_VOLUME
    };

    void inject_frame(InjectVolumeStatus status)
    {
        //std::cout << "flag \"inject_frame\" updated!!!";
        f_inject_frame.store(status == INJECT_FRAME);
    }

    bool inject_frame()
    {
        return f_inject_frame.load();
    }


};


enum WindowSpecification
{
    ROI_SPECIFICATION,
    VOLUME_RECONSTRUCTION
};

struct ApplicationState
{
    WindowSpecification specification{ROI_SPECIFICATION};
    bool operation_in_progress = false;
    bool show_error = false;
    bool show_sucess = false;
    std::mutex mut;
    std::string operation_description;
    std::string success_description;
    ImVec2 padding{0, 40};
    std::shared_ptr<curan::utilities::ThreadPool> pool;
    RobotState robot_state;
    std::string filename{CURAN_COPIED_RESOURCE_PATH "/reconstruction_results.mha"};

    ApplicationState(curan::renderable::Window &wind) : robot_state{wind}
    {
        /*
        We need three threads:
        1) one for volumetric reconstruction
        2) another for the communication thread
        3) Another to process the UI tasks in sequence
        */
        pool = curan::utilities::ThreadPool::create(3); 
    }

    void showMainWindow()
    {
        ImGui::Begin("Volume Reconstruction", NULL, ImGuiWindowFlags_MenuBar);
        if (ImGui::BeginMenuBar())
        {
            if (ImGui::BeginMenu("Mode"))
            {
                if (ImGui::MenuItem("ROI specification", "Ctrl+O"))
                {
                    if (operation_in_progress && specification != ROI_SPECIFICATION)
                    {
                        std::lock_guard<std::mutex> g{mut};
                        show_error = true;
                    }
                    else
                        specification = ROI_SPECIFICATION;
                }
                if (ImGui::MenuItem("Reconstruction", "Ctrl+S"))
                {
                    if (operation_in_progress && specification != VOLUME_RECONSTRUCTION)
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
        ImGui::Dummy(padding);
        ImGui::SameLine();
        bool local_copy;
        {
            std::lock_guard<std::mutex> g{mut};
            local_copy = operation_in_progress;
        }
        if (local_copy)
            ImGui::ProgressBar(-1.0f * ImGui::GetTime());
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextWrapped("You can select a region of interest and then inject B-Scans into the volume. You always need to specify the bounding box before reconstruction"); // Display some text (you can use a format strings too)
        ImGui::Dummy(padding);
        ImGui::SameLine();

        switch (specification)
        {
        case ROI_SPECIFICATION:
            showRegionOfInterestWindow();
            break;
        case VOLUME_RECONSTRUCTION:
        default:
            showReconstructionWindow();
            break;
        }

        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
        showOverlayErrorWindow();
        showOverlaySuccessWindow();
    }

    void showReconstructionWindow()
    {
        ImGui::Dummy(padding); ImGui::SameLine();
        {
            std::lock_guard<std::mutex> g{mut};
            ImGui::InputText("Filename",&filename);
        }
        
        if (ImGui::Button("Save Volume"))
        {
            {
                std::lock_guard<std::mutex> g{mut};
                operation_in_progress = true;
                operation_description = "saving volumetric reconstruction";
                robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
            }
            pool->submit(curan::utilities::Job{"testing", [this]()
                                               {
                                                   nlohmann::json specified_box;
                                                   std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/specified_box.json");
                                                   in >> specified_box;

                                                   std::string timestamp_box = specified_box["timestamp"];

                                                   std::stringstream spacing_box_info;
                                                   std::string spacing_box = specified_box["spacing"];
                                                   spacing_box_info << spacing_box;
                                                   Eigen::MatrixXd spacing = curan::utilities::convert_matrix(spacing_box_info, ',');
                                                   std::string origin_box = specified_box["origin"];
                                                   std::stringstream origin_box_info;
                                                   origin_box_info << origin_box;
                                                   Eigen::MatrixXd origin = curan::utilities::convert_matrix(origin_box_info, ',');
                                                   std::string size_box = specified_box["size"];
                                                   std::stringstream size_box_info;
                                                   size_box_info << size_box;

                                                   Eigen::MatrixXd size = curan::utilities::convert_matrix(size_box_info, ',');
                                                   std::string direction_box = specified_box["direction"];
                                                   std::stringstream direction_box_info;
                                                   direction_box_info << direction_box;

                                                   Eigen::MatrixXd direction = curan::utilities::convert_matrix(direction_box_info, ',');

                                                   if (spacing.rows() != 1 || spacing.cols() != 3)
                                                       throw std::runtime_error("The supplied spacing has an incorrect dimension (expected : [1x3])");

                                                   if (origin.rows() != 1 || origin.cols() != 3)
                                                       throw std::runtime_error("The supplied origin has an incorrect dimension (expected : [1x3])");

                                                   if (size.rows() != 1 || size.cols() != 3)
                                                       throw std::runtime_error("The supplied size has an incorrect dimension (expected : [1x3])");

                                                   if (direction.rows() != 3 || direction.cols() != 3)
                                                       throw std::runtime_error("The supplied direction has an incorrect dimension (expected : [3x3])");

                                                   itk::Size<3U> output_size;
                                                   output_size = robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_output_size();

                                                   using PixelType = float;
                                                   using ImageType = itk::Image<PixelType, 3>;
                                                   ImageType::Pointer itkVolume = ImageType::New();

                                                   using ImportFilterType = itk::ImportImageFilter<PixelType, 3>;
                                                   auto importFilter = ImportFilterType::New();
                                                   ImportFilterType::IndexType start;
                                                   start.Fill(0);

                                                   ImportFilterType::RegionType region;
                                                   region.SetIndex(start);
                                                   region.SetSize(output_size);

                                                   importFilter->SetRegion(region);

                                                   const itk::SpacePrecisionType output_origin[3] = {origin(0, 0) * 1000, origin(0, 1) * 1000, origin(0, 2) * 1000};
                                                   importFilter->SetOrigin(output_origin);

                                                   const itk::SpacePrecisionType output_spacing[3] = {spacing(0, 0) * 1000, spacing(0, 1) * 1000, spacing(0, 2) * 1000};
                                                   importFilter->SetSpacing(output_spacing);
                                                   const unsigned int numberOfPixels = output_size[0] * output_size[1] * output_size[2];

                                                   itk::Matrix<double> orientation;
                                                   for (size_t row = 0; row < 3; ++row)
                                                       for (size_t col = 0; col < 3; ++col)
                                                           orientation(row, col) = direction(row, col);
                                                   importFilter->SetDirection(orientation);

                                                   const bool importImageFilterWillOwnTheBuffer = false;
                                                   if(robot_state.integrated_volume.get()==nullptr)
                                                    return;
                                                   float *my_beatiful_pointer = robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->get_texture_data()->data();
                                                   importFilter->SetImportPointer(my_beatiful_pointer, numberOfPixels, importImageFilterWillOwnTheBuffer);

                                                   using WriterType = itk::ImageFileWriter<ImageType>;
                                                   WriterType::Pointer writer = WriterType::New();

                                                   {
                                                       std::lock_guard<std::mutex> g{mut};
                                                       writer->SetFileName(filename);
                                                   }
                                                   writer->SetInput(importFilter->GetOutput());
                                                   writer->Update();

                                                   {
                                                       std::lock_guard<std::mutex> g{mut};
                                                       success_description = "saved volume information";
                                                       operation_in_progress = false;
                                                       show_sucess = true;
                                                       robot_state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
                                                       
                                                   }
                                               }});
        }
    };

    void showRegionOfInterestWindow()
    {
        static bool local_record_data = false;
        static bool previous_local_record_data = local_record_data;
        ImGui::Checkbox("Start Frame Collection", &local_record_data);
        if (local_record_data)
        {
            std::lock_guard<std::mutex> g{mut};
            operation_in_progress = true;
            operation_description = "collecting data from ultrasound for region of interest";
            robot_state.record_frames(RobotState::START_RECORDING_FOR_BOX_UPDATE);
            robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
            if (previous_local_record_data != local_record_data)
            {
                robot_state.box_class.reset();
            }
        }
        else
            robot_state.record_frames(RobotState::NOT_RECORDING);
        previous_local_record_data = local_record_data;
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Save Region of Interest"))
        {
            robot_state.record_frames(RobotState::NOT_RECORDING);
            local_record_data = false;
            operation_in_progress = true;
            operation_description = "saving region of interest";

            pool->submit(curan::utilities::Job{"testing", [this]()
                                               {
                                                   auto final_box = robot_state.box_class.get_final_volume_vertices();
                                                   vsg::dmat3 rotation_0_1;
                                                   Eigen::Matrix<double, 3, 3> final_box_orientation;
                                                   for (size_t col = 0; col < 3; ++col)
                                                       for (size_t row = 0; row < 3; ++row)
                                                       {
                                                           rotation_0_1(col, row) = final_box.axis[col][row];
                                                           final_box_orientation(row, col) = final_box.axis[col][row];
                                                       }

                                                   vsg::dvec3 position_of_center_in_global_frame;
                                                   position_of_center_in_global_frame[0] = final_box.center[0];
                                                   position_of_center_in_global_frame[1] = final_box.center[1];
                                                   position_of_center_in_global_frame[2] = final_box.center[2];

                                                   vsg::dvec3 position_in_local_box_frame;
                                                   position_in_local_box_frame[0] = final_box.extent[0];
                                                   position_in_local_box_frame[1] = final_box.extent[1];
                                                   position_in_local_box_frame[2] = final_box.extent[2];

                                                   auto global_corner_position = position_of_center_in_global_frame - rotation_0_1 * position_in_local_box_frame;
                                                   nlohmann::json specified_box;
                                                   specified_box["timestamp"] = return_current_time_and_date();

                                                   constexpr size_t maximum_float_size = 62.5e6 * 0.5;
                                                   double new_spacing = std::cbrt((2 * final_box.extent[0] * 2 * final_box.extent[1] * 2 * final_box.extent[2]) / (maximum_float_size));

                                                   std::stringstream ss;
                                                   ss << new_spacing << " , " << new_spacing << " , " << new_spacing;
                                                   specified_box["spacing"] = ss.str();
                                                   ss.str("");
                                                   ss << global_corner_position[0] << " , " << global_corner_position[1] << " , " << global_corner_position[2];
                                                   specified_box["origin"] = ss.str();
                                                   ss.str("");
                                                   ss << 2 * final_box.extent[0] << " , " << 2 * final_box.extent[1] << " , " << 2 * final_box.extent[2];
                                                   specified_box["size"] = ss.str();
                                                   ss.str("");
                                                   Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");
                                                   ss << final_box_orientation.format(CleanFmt);
                                                   specified_box["direction"] = ss.str();

                                                   // write prettified JSON to another file
                                                   std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/specified_box.json");
                                                   o << specified_box;

                                                   std::string timestamp_box = specified_box["timestamp"];

                                                   std::stringstream spacing_box_info;
                                                   std::string spacing_box = specified_box["spacing"];
                                                   spacing_box_info << spacing_box;
                                                   Eigen::MatrixXd spacing = curan::utilities::convert_matrix(spacing_box_info, ',');
                                                   std::string origin_box = specified_box["origin"];
                                                   std::stringstream origin_box_info;
                                                   origin_box_info << origin_box;
                                                   Eigen::MatrixXd origin = curan::utilities::convert_matrix(origin_box_info, ',');
                                                   std::string size_box = specified_box["size"];
                                                   std::stringstream size_box_info;
                                                   size_box_info << size_box;

                                                   Eigen::MatrixXd size = curan::utilities::convert_matrix(size_box_info, ',');
                                                   std::string direction_box = specified_box["direction"];
                                                   std::stringstream direction_box_info;
                                                   direction_box_info << direction_box;

                                                   Eigen::MatrixXd direction = curan::utilities::convert_matrix(direction_box_info, ',');

                                                   if (spacing.rows() != 1 || spacing.cols() != 3)
                                                       throw std::runtime_error("The supplied spacing has an incorrect dimension (expected : [1x3])");

                                                   if (origin.rows() != 1 || origin.cols() != 3)
                                                       throw std::runtime_error("The supplied origin has an incorrect dimension (expected : [1x3])");

                                                   if (size.rows() != 1 || size.cols() != 3)
                                                       throw std::runtime_error("The supplied size has an incorrect dimension (expected : [1x3])");

                                                   if (direction.rows() != 3 || direction.cols() != 3)
                                                       throw std::runtime_error("The supplied direction has an incorrect dimension (expected : [3x3])");

                                                   std::array<double, 3> vol_origin = {origin(0, 0), origin(0, 1), origin(0, 2)};
                                                   std::array<double, 3> vol_spacing = {spacing(0, 0), spacing(0, 1), spacing(0, 2)};
                                                   std::array<double, 3> vol_size = {size(0, 0), size(0, 1), size(0, 2)};
                                                   std::array<std::array<double, 3>, 3> vol_direction;
                                                   vol_direction[0] = {direction(0, 0), direction(1, 0), direction(2, 0)};
                                                   vol_direction[1] = {direction(0, 1), direction(1, 1), direction(2, 1)};
                                                   vol_direction[2] = {direction(0, 2), direction(1, 2), direction(2, 2)};
                                                   curan::image::IntegratedReconstructor::Info recon_info{vol_spacing, vol_origin, vol_size, vol_direction};
                                                   recon_info.identifier = "reconstructor";
                                                   
                                                   robot_state.integrated_volume_create_info = recon_info;
                                                   robot_state.generate_volume(RobotState::GenerateStatus::GENERATE_VOLUME);
                                                   robot_state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
                                                   std::lock_guard<std::mutex> g{mut};
                                                   success_description = "saved volume information";
                                                   operation_in_progress = false;
                                                   show_sucess = true;
                                               }});
        }
    };

    void showOverlayErrorWindow()
    {
        if (!show_error)
            return;
        ImGui::Begin("Error Reporting", NULL, ImGuiWindowFlags_MenuBar);
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0, 0, 1), "Operation already in progress...");
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1, 0, 0, 1), operation_description.data());
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Ok!"))
        {
            show_error = false;
        }
        ImGui::End();
    };

    void showOverlaySuccessWindow()
    {
        if (!show_sucess)
            return;
        ImGui::Begin("Operation Sucessefull", NULL, ImGuiWindowFlags_MenuBar);
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Operation finished");
        ImGui::Dummy(padding);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), success_description.data());
        ImGui::Dummy(padding);
        ImGui::SameLine();
        if (ImGui::Button("Ok!"))
        {
            show_sucess = false;
        }
        ImGui::End();
    };
};


bool process_image_message(RobotState &state, igtl::MessageBase::Pointer val)
{
    igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
    message_body->Copy(val);
    int c = message_body->Unpack(1);
    if (!(c & igtl::MessageHeader::UNPACK_BODY))
        return false; // failed to unpack message, therefore returning without doing anything
    if (!state.dynamic_texture)
    {
        std::cout << "creating dynamic texture\n";
        int x, y, z;
        message_body->GetDimensions(x, y, z);
        curan::renderable::DynamicTexture::Info infotexture;
        infotexture.height = y;
        infotexture.width = x;
        infotexture.spacing = {0.00018867924, 0.00018867924, 0.00018867924};
        infotexture.origin = {0.0, 0.0, 0.0};
        infotexture.identifier = "ultrasound";
        infotexture.builder = vsg::Builder::create();
        state.dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
        state.window_pointer << *state.dynamic_texture;

        std::cout << "creating bounding box texture\n";
        curan::renderable::Box::Info infobox;
        infobox.builder = vsg::Builder::create();
        infobox.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
        infobox.geomInfo.dx = vsg::vec3(1.0f, 0.0, 0.0);
        infobox.geomInfo.dy = vsg::vec3(0.0, 1.0f, 0.0);
        infobox.geomInfo.dz = vsg::vec3(0.0, 0.0, 1.0f);
        infobox.identifier = "roi";
        infobox.stateInfo.wireframe = true;
        infobox.geomInfo.position = vsg::vec3(0.5, 0.5, 0.5);
        state.rendered_box = curan::renderable::Box::make(infobox);
        state.window_pointer << state.rendered_box;
        std::cout << "creating box and ultrasound\n";
    }

    auto updateBaseTexture = [message_body](vsg::vec4Array2D &image)
    {
        try
        {
            int x, y, z;
            message_body->GetDimensions(x, y, z);
            unsigned char *scaller_buffer = (unsigned char *)message_body->GetScalarPointer();

            for (size_t r = 0; r < image.height(); ++r)
            {
                using value_type = typename vsg::vec4Array2D::value_type;
                value_type *ptr = &image.at(0, r);
                for (size_t c = 0; c < image.width(); ++c)
                {
                    auto val = *scaller_buffer / 255.0;
                    ptr->r = val;
                    ptr->g = val;
                    ptr->b = val;
                    ptr->a = 1.0f;
                    ++ptr;
                    ++scaller_buffer;
                }
            }
        }
        catch (std::exception &e)
        {
            std::cout << "exception : " << e.what() << std::endl;
        }
    };

    //if(state.dynamic_texture->get()!=nullptr)
    //    state.dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_texture(updateBaseTexture);
    igtl::Matrix4x4 image_transform;
    message_body->GetMatrix(image_transform);
    vsg::dmat4 homogeneous_transformation;
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            homogeneous_transformation(col, row) = image_transform[row][col];

    homogeneous_transformation(3, 0) *= 1e-3;
    homogeneous_transformation(3, 1) *= 1e-3;
    homogeneous_transformation(3, 2) *= 1e-3;
    auto product = homogeneous_transformation * state.calibration_matrix;

    if(state.dynamic_texture->get()!=nullptr)
        state.dynamic_texture->cast<curan::renderable::DynamicTexture>()->update_transform(product);

    OutputImageType::Pointer image_to_render;
    curan::image::igtl2ITK_im_convert(message_body, image_to_render);
    const itk::SpacePrecisionType spacing[3] = {0.00018867924, 0.00018867924, 0.00018867924};
    image_to_render->SetSpacing(spacing);

    itk::Matrix<double, 3, 3> itk_matrix;
    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            itk_matrix(row, col) = product(col, row);

    image_to_render->SetDirection(itk_matrix);
    auto origin = image_to_render->GetOrigin();
    origin[0] = product(3, 0);
    origin[1] = product(3, 1);
    origin[2] = product(3, 2);
    image_to_render->SetOrigin(origin);

    static size_t counter = 0;
    constexpr size_t update_rate = 2;
    ++counter;

    if (state.record_frames() && (counter % update_rate == 0))
    {
        state.box_class.add_frame(image_to_render);
        state.box_class.update();
    }

    auto caixa = state.box_class.get_final_volume_vertices();
    vsg::dmat4 transform_matrix;

    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
            transform_matrix(col, row) = image_to_render->GetDirection()[row][col];

    transform_matrix(3, 0) = image_to_render->GetOrigin()[0];
    transform_matrix(3, 1) = image_to_render->GetOrigin()[1];
    transform_matrix(3, 2) = image_to_render->GetOrigin()[2];

    vsg::dmat3 rotation_0_1;

    vsg::dmat4 box_transform_matrix = vsg::translate(0.0, 0.0, 0.0);

    for (size_t col = 0; col < 3; ++col)
        for (size_t row = 0; row < 3; ++row)
        {
            box_transform_matrix(col, row) = caixa.axis[col][row];
            rotation_0_1(col, row) = box_transform_matrix(col, row);
        }

    vsg::dvec3 position_of_center_in_global_frame;
    position_of_center_in_global_frame[0] = caixa.center[0];
    position_of_center_in_global_frame[1] = caixa.center[1];
    position_of_center_in_global_frame[2] = caixa.center[2];

    vsg::dvec3 position_in_local_box_frame;
    position_in_local_box_frame[0] = caixa.extent[0];
    position_in_local_box_frame[1] = caixa.extent[1];
    position_in_local_box_frame[2] = caixa.extent[2];

    auto global_corner_position = position_of_center_in_global_frame - rotation_0_1 * position_in_local_box_frame;

    box_transform_matrix(3, 0) = global_corner_position[0];
    box_transform_matrix(3, 1) = global_corner_position[1];
    box_transform_matrix(3, 2) = global_corner_position[2];
    box_transform_matrix(3, 3) = 1;

    state.rendered_box->cast<curan::renderable::Box>()->set_scale(caixa.extent[0] * 2, caixa.extent[1] * 2, caixa.extent[2] * 2);
    state.rendered_box->update_transform(box_transform_matrix);

    if(state.generate_volume()){
        state.inject_frame(RobotState::InjectVolumeStatus::FREEZE_VOLUME);
        state.window_pointer.erase("reconstructor");
        
        int clip_origin_x = (int)0;
        int clip_origin_y = (int)0;
        int x, y, z;
	    message_body->GetDimensions(x, y, z);

        state.integrated_volume = curan::image::IntegratedReconstructor::make(state.integrated_volume_create_info);
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_compound(curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE).set_interpolation(curan::image::reconstruction::Interpolation::NEAREST_NEIGHBOR_INTERPOLATION);
        state.window_pointer << state.integrated_volume;
        
        curan::image::Clipping desired_clip;
        desired_clip.clipRectangleOrigin[0] = clip_origin_x;
        desired_clip.clipRectangleOrigin[1] = clip_origin_y;
        desired_clip.clipRectangleSize[0] = x;
        desired_clip.clipRectangleSize[1] = y-15;
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->set_clipping(desired_clip);
        
        state.generate_volume(RobotState::GenerateStatus::ALREADY_GENERATED);
        state.inject_frame(RobotState::InjectVolumeStatus::INJECT_FRAME);
    }

    if(state.integrated_volume.get()!=nullptr && state.inject_frame()){
        state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->add_frame(image_to_render);
    }
    return true;
}

std::map<std::string, std::function<bool(RobotState &state, igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
    {"IMAGE", process_image_message}};

bool process_message(RobotState &state, size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
    assert(val.IsNotNull());
    if (er)
        return true;
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(state, val);
    return false;
}



std::ofstream& get_file_handle(){
    static bool initializing = true;
    static std::string pathname = std::string{"joint_recording_"}+return_current_time_and_date()+std::string{".txt"};
    static std::ofstream out{pathname};
    if(initializing){
        if(!out.is_open())
            std::cout << "failed to create file with name:" << pathname << std::endl;
        else
            std::cout << "printing to file" << pathname << std::endl;
    }
    initializing = false;
    return out;
}

bool process_joint_message(RobotState &state, const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
    auto& handle = get_file_handle();
    if (er)
        return true;
    for (size_t joint_index = 0; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index){
        state.robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, message->angles[joint_index]);
        handle << message->angles[joint_index] << " ";
    }
    handle << "\n";
    handle.flush();
    return false;
}

int communication(RobotState &state, asio::io_context &context)
{
    asio::ip::tcp::resolver resolver(context);
    auto client = curan::communication::Client<curan::communication::protocols::igtlink>::make(context,resolver.resolve("localhost", std::to_string(18944)));

    auto lam = [&](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
    {
        try
        {
            if (process_message(state, protocol_defined_val, er, val) || state)
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    client->connect(lam);
    std::cout << "connecting to client\n";
    
    asio::ip::tcp::resolver fri_resolver(context);
    auto fri_client = curan::communication::Client<curan::communication::protocols::fri>::make(context,fri_resolver.resolve("172.31.1.148", std::to_string(50010)));

    auto lam_fri = [&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
    {
        try
        {
            if (process_joint_message(state, protocol_defined_val, er, message))
                context.stop();
        }
        catch (...)
        {
            std::cout << "Exception was thrown\n";
        }
    };
    fri_client->connect(lam_fri);
    
    context.run();
    std::cout << "stopped connecting to client\n";
    return 0;
}

void interface(vsg::CommandBuffer &cb,ApplicationState** app)
{
    if(*app!=nullptr)
        (**app).showMainWindow();
}

int main(int argc, char **argv)
{
    asio::io_context context;
    ApplicationState* app_pointer = nullptr;

    curan::renderable::ImGUIInterface::Info info_gui{[pointer_to_address = &app_pointer](vsg::CommandBuffer &cb){interface(cb,pointer_to_address);}};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "Volume Reconstructor";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{2000, 1800};
    info.window_size = size;
    curan::renderable::Window window{info};

    ApplicationState application_state{window};
    app_pointer = &application_state;

    curan::utilities::UltrasoundCalibrationData calibration{CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json"};

    for (Eigen::Index row = 0; row < 4; ++row)
        for (Eigen::Index col = 0; col < 4; ++col)
            application_state.robot_state.calibration_matrix(col, row) = calibration.homogeneous_transformation()(row,col);
    
   
    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    application_state.robot_state.robot = curan::renderable::SequencialLinks::make(create_info);
    window << application_state.robot_state.robot;

    application_state.pool->submit(curan::utilities::Job{"communication with robot",[&](){communication(application_state.robot_state,context);}});
    application_state.pool->submit(curan::utilities::Job{"reconstruct volume",[&](){
        auto reconstruction_thread_pool = curan::utilities::ThreadPool::create(8);
        while(!context.stopped()){
            if(application_state.robot_state.inject_frame() && application_state.robot_state.integrated_volume.get()!=nullptr)
                application_state.robot_state.integrated_volume->cast<curan::image::IntegratedReconstructor>()->multithreaded_update(reconstruction_thread_pool);
        }
    }});

    window.run();
    context.stop();
    get_file_handle().close();
    std::cout << "closed file handle";
    return 0;
}
