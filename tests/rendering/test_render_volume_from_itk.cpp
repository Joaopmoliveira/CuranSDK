#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/Sphere.h"
#include <iostream>

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkRescaleIntensityImageFilter.h>
#include <itkCastImageFilter.h>

template <typename itkImage>
void updateBaseTexture3D(vsg::floatArray3D &image, typename itkImage::Pointer out)
{
    typename itk::ImageRegionIteratorWithIndex<itkImage> outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
        image.set(outputIt.GetIndex()[0], outputIt.GetIndex()[1], outputIt.GetIndex()[2], outputIt.Get());
    image.dirty();
}

int main(int argc, char **argv)
{
    try
    {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::printf("\nReading input volume...\n");
        auto fixedImageReader = itk::ImageFileReader<itk::Image<double, 3>>::New();
        //fixedImageReader->SetFileName(CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha");
        fixedImageReader->SetFileName("C:/Dev/CuranSDK/build/release/bin/resources/original_volume.mha");
        

        // Rescale and cast the volume to use with the correct MaskPixelType (0-255)
        auto rescale = itk::RescaleIntensityImageFilter<itk::Image<double, 3>, itk::Image<double, 3>>::New();
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

        volume->cast<curan::renderable::Volume>()->update_volume([=](vsg::floatArray3D &image)
                                                                 { updateBaseTexture3D<itk::Image<double, 3>>(image, output); });

        curan::renderable::Sphere::Info infosphere;
        infosphere.builder = vsg::Builder::create();
        infosphere.geomInfo.color = vsg::vec4(1.0, 0.0, 0.0, 1.0);
        infosphere.geomInfo.dx = vsg::vec3(0.01f, 0.0, 0.0);
        infosphere.geomInfo.dy = vsg::vec3(0.0, 0.01f, 0.0);
        infosphere.geomInfo.dz = vsg::vec3(0.0, 0.0, 0.01f);
        infosphere.stateInfo.blending = true;
        auto origin = curan::renderable::Sphere::make(infosphere);

        window << origin;

        window.run();

        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map)
            {
                for (auto &p : map)
                {
                    std::cout << "Object contained: " << p.first << '\n';
                }
            });
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    // clean up done automatically thanks to ref_ptr<>
    return 0;
}