#include "rendering/ImGUIInterface.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include <iostream>


#include <itkCastImageFilter.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkRescaleIntensityImageFilter.h>


template <typename itkImage>
void updateBaseTexture3D(vsg::floatArray3D &image,
                         typename itkImage::Pointer out) {
  typename itk::ImageRegionIteratorWithIndex<itkImage> outputIt(
      out, out->GetRequestedRegion());
  for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    image.set(outputIt.GetIndex()[0], outputIt.GetIndex()[1],
              outputIt.GetIndex()[2], outputIt.Get());
  image.dirty();
}

void interface(vsg::CommandBuffer &cb) {
  ImGui::Begin("Volume Registration", NULL, ImGuiWindowFlags_MenuBar);
  ImGui::TextWrapped(
      "To validate your registration procedure you can press \"Register\" "
      "which will solve the registration problem. Henceforth validade if the estimated solution "
      "is correct"); // Display some text (you can use a format strings too)
  if (ImGui::Button("Register")) {
    std::cout << "solve registration\n";
  }
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

int main(int argc, char **argv) {
  try {

    curan::renderable::ImGUIInterface::Info info_gui{interface};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "Volume Registration";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{2000, 1800};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::printf("\nReading input volume...\n");
    auto fixedImageReader = itk::ImageFileReader<itk::Image<double, 3>>::New();
    fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha");

    // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
    auto rescale =
        itk::RescaleIntensityImageFilter<itk::Image<double, 3>,
                                         itk::Image<double, 3>>::New();
    rescale->SetInput(fixedImageReader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(1.0);

    rescale->Update();

    itk::Image<double, 3>::Pointer output = rescale->GetOutput();
    auto region = output->GetLargestPossibleRegion();
    auto size_itk = region.GetSize();
    auto spacing = output->GetSpacing();

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size_itk.GetSize()[0];
    volumeinfo.height = size_itk.GetSize()[1];
    volumeinfo.depth = size_itk.GetSize()[2];
    volumeinfo.spacing_x = spacing[0];
    volumeinfo.spacing_y = spacing[1];
    volumeinfo.spacing_z = spacing[2];
    auto volume = curan::renderable::Volume::make(volumeinfo);
    window << volume;

    auto mat = vsg::translate(-1.0, 0.0, 0.3);
    volume->update_transform(mat);

    volume->cast<curan::renderable::Volume>()->update_volume(
        [=](vsg::floatArray3D &image) {
          updateBaseTexture3D<itk::Image<double, 3>>(image, output);
        });

    std::filesystem::path robot_path =
        CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    vsg::ref_ptr<curan::renderable::Renderable> robotRenderable =
        curan::renderable::SequencialLinks::make(create_info);
    window << robotRenderable;

    robotRenderable->cast<curan::renderable::SequencialLinks>()->set(3, 1.0);
    robotRenderable->cast<curan::renderable::SequencialLinks>()->set(1, -1.0);

    window.run();

    window.transverse_identifiers(
        [](const std::unordered_map<
            std::string, vsg::ref_ptr<curan::renderable::Renderable>> &map) {
          for (auto &p : map) {
            std::cout << "Object contained: " << p.first << '\n';
          }
        });
  } catch (const std::exception &e) {
    std::cerr << "Exception thrown : " << e.what() << std::endl;
    return 1;
  }
  // clean up done automatically thanks to ref_ptr<>
  return 0;
}