#include <iostream>
#include <string>
#include <sstream>
#include <charconv>
#include <stdexcept>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageDuplicator.h"
#include "itkRegionOfInterestImageFilter.h"

/*The program takes 3 input arguments: the input volume, the start coordinate and the end coordinate. Example:
./test_subvolume_generator ultrasound_precious_phantom1.mha  0,100,0 454,244,567
It uses the post build resource path, so make sure to put the input files in that directory. 
Then, by default, it creates the output volume in this path and names it according to the choosen coordinates
to make the process easy when experimenting with lots of subvolumes.*/

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using WriterType = itk::ImageFileWriter<ImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;

int main(int argc, char* argv[]){
    if(argc!=4){
        std::cout << "To run you must provide the input volume, starting and ending coordinates separated by spaces. Example:precious_phantom.mha 0,0,0 150,254,140\n";
        return 1;
    }

    std::string input_volume{argv[1]};
    std::string close_to_origin{argv[2]};
    std::string away_from_origin{argv[3]};
    std::vector<int> values_start;
    {
    std::istringstream ss(close_to_origin);
    for (std::string line; std::getline(ss, line,',');){
        int result{};
        auto [ptr, ec] = std::from_chars(line.data(), line.data() + line.size(), result);
        if (ec == std::errc::invalid_argument){
            std::cout << "This is not a number.\n";
            return 1;
        }
        else if (ec == std::errc::result_out_of_range){
            std::cout << "This number is larger than an int.\n";
            return 1;
        }
        values_start.push_back(result);
    }
    if(values_start.size()!=3){
        std::cout << "Must supply three coordinate values.\n";
        return 1;
    }
    }

    std::vector<int> values_end;
    std::istringstream ss(away_from_origin);
    for (std::string line; std::getline(ss, line,',');){
        int result{};
        auto [ptr, ec] = std::from_chars(line.data(), line.data() + line.size(), result);
        if (ec == std::errc::invalid_argument){
            std::cout << "This is not a number.\n";
            return 1;
        }
        else if (ec == std::errc::result_out_of_range){
            std::cout << "This number is larger than an int.\n";
            return 1;
        }
        values_end.push_back(result);
    }
    if(values_end.size()!=3){
        std::cout << "Must supply three coordinate values.\n";
        return 1;
    }
    
    for(int i = 0; i< 3; ++i){
        if(values_end[i] <= values_start[i]){
            std::printf("the (%d) ending coordinates (%d) must be larger than the start coordinates (%d)\n",i,values_end[i],values_start[i]);
            return 1;
        }
    }
    
    auto fixedImageReader = FixedImageReaderType::New();
    const std::string aux = "/precious_phantom/";
    const std::string path = CURAN_COPIED_RESOURCE_PATH + aux + input_volume;
    if(true)
        fixedImageReader->SetFileName(input_volume);
    else
        fixedImageReader->SetFileName(path);
    try {
        fixedImageReader->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    using DuplicatorType = itk::ImageDuplicator<ImageType>;
    auto duplicator = DuplicatorType::New();
    duplicator->SetInputImage(fixedImageReader->GetOutput());


    try {
        duplicator->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        throw std::runtime_error("error");
    }

    ImageType::Pointer new_image = duplicator->GetOutput();
    ImageType::SizeType size_itk = new_image->GetLargestPossibleRegion().GetSize();
    std::cout << "Original Image size:" << size_itk[0] << "x" << size_itk[1] << "x" << size_itk[2] << std::endl;

    ImageType::IndexType start;
    start[0] = values_start[0];
    start[1] = values_start[1];
    start[2] = values_start[2];

    ImageType::IndexType end;
    end[0] = values_end[0];
    end[1] = values_end[1];
    end[2] = values_end[2];

    ImageType::SizeType size;
    size[0] = end[0] - start[0];
    size[1] = end[1] - start[1];
    size[2] = end[2] - start[2];

    ImageType::RegionType region;
    region.SetIndex(start);
    region.SetSize(size);

    using FilterType = itk::RegionOfInterestImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(new_image);
    filter->SetRegionOfInterest(region);

    std::string output_volume = CURAN_COPIED_RESOURCE_PATH + aux + "output_volume_" +
    std::to_string(values_start[0]) + "," + 
    std::to_string(values_start[1]) + "," +
    std::to_string(values_start[2]) + "_" +
    std::to_string(values_end[0]) + "," +
    std::to_string(values_end[1]) + "," +
    std::to_string(values_end[2]) + ".mha";

    using WriterType = itk::ImageFileWriter<ImageType>;
    WriterType::Pointer writer = WriterType::New();
    writer->SetInput(filter->GetOutput());
    writer->SetFileName(output_volume);

    try {
       writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        throw std::runtime_error("error");
    }

    std::cout << "File written to " << output_volume << std::endl;

    return 0;
}