#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"

using PixelType = signed short;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

void updateBaseTexture3D(vsg::floatArray3D& image, ImageType::Pointer image_to_render)
{
    using OutputPixelType = float;
    using InputImageType = itk::Image<PixelType, Dimension>;
    using OutputImageType = itk::Image<OutputPixelType, Dimension>;
    using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<OutputImageType, OutputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try{
        rescale->Update();
    } catch (const itk::ExceptionObject& e) {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    OutputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<OutputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt){
        ImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}


int main(int argc, char** argv) {
try{
    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");
    nameGenerator->SetGlobalWarningDisplay(false);
    std::string dirName{CURAN_COPIED_RESOURCE_PATH"/dicom_sample/mri_brain"};
    nameGenerator->SetDirectory(dirName);

    ImageType::Pointer image_to_render;

    try{
        using SeriesIdContainer = std::vector<std::string>;
        const SeriesIdContainer& seriesUID = nameGenerator->GetSeriesUIDs();
        auto  seriesItr = seriesUID.begin();
        auto  seriesEnd = seriesUID.end();

        if (seriesItr == seriesEnd)
        {
            std::cout << "No DICOMs in: " << dirName << std::endl;
            return EXIT_SUCCESS;
        }

        seriesItr = seriesUID.begin();
        while (seriesItr != seriesUID.end())
        {
            std::string seriesIdentifier;
            if (argc > 3)
            {
                seriesIdentifier = argv[3];
                seriesItr = seriesUID.end();
            }
            else 
            {
                seriesIdentifier = seriesItr->c_str();
                seriesItr++;
            }
            using FileNamesContainer = std::vector<std::string>;
            FileNamesContainer fileNames = nameGenerator->GetFileNames(seriesIdentifier);

            using ReaderType = itk::ImageSeriesReader<ImageType>;
            auto reader = ReaderType::New();
            using ImageIOType = itk::GDCMImageIO;
            auto dicomIO = ImageIOType::New();
            reader->SetImageIO(dicomIO);
            reader->SetFileNames(fileNames);
            reader->ForceOrthogonalDirectionOff(); // properly read CTs with gantry tilt

            try
            {
                reader->Update();
                image_to_render = reader->GetOutput();
            }
            catch (const itk::ExceptionObject& ex)
            {
                std::cout << ex << std::endl;
                continue;
            }
        }
    }
    catch (const itk::ExceptionObject& ex)
    {
        std::cout << ex << std::endl;
        return EXIT_FAILURE;
    }
    ImageType::RegionType region = image_to_render->GetLargestPossibleRegion();
    ImageType::SizeType size_itk = region.GetSize();
    ImageType::SpacingType spacing = image_to_render->GetSpacing();
    
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

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size_itk.GetSize()[0]; 
    volumeinfo.height = size_itk.GetSize()[1];
    volumeinfo.depth = size_itk.GetSize()[2];
    volumeinfo.spacing_x = spacing[0];
    volumeinfo.spacing_y = spacing[1];
    volumeinfo.spacing_z = spacing[2];
    auto volume = curan::renderable::Volume::make(volumeinfo);
    window << volume;

    auto casted_volume = volume->cast<curan::renderable::Volume>();
    auto updater = [image_to_render](vsg::floatArray3D& image){
        updateBaseTexture3D(image, image_to_render);
    };
    casted_volume->update_texture(updater);

    std::atomic<bool> continue_moving = false;
    auto mover = [&continue_moving,volume](){
        double time = 0.0;
        while(continue_moving.load()){
            volume->update_transform(vsg::translate(std::cos(time)*0.2,std::sin(time)*0.2,0.2)*vsg::rotate(time,1.0,0.0,0.0));
            time += 0.016;
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
    };
    std::thread mover_thread{mover};

    window.run();
    continue_moving.store(false);
    mover_thread.join();

    window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable >>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }
    });

} catch (const std::exception& e) {
     std::cerr << "Exception thrown : " << e.what() << std::endl;
    return 1;
}
// clean up done automatically thanks to ref_ptr<>
return 0;

}