#include <iostream>
#include <optional>
#include <string>
#include <sstream>
#include <cassert>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <system_error>

#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkImageRegistrationMethodv4.h"
#include "itkMeanSquaresImageToImageMetricv4.h"
#include "itkEuler3DTransform.h"
#include "itkMattesMutualInformationImageToImageMetricv4.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkRegularStepGradientDescentOptimizerv4.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkCommand.h"
#include "itkImageDuplicator.h"
#include "itkRegionOfInterestImageFilter.h"


using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using TransformType = itk::Euler3DTransform<double>;
using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
using MetricType = itk::MattesMutualInformationImageToImageMetricv4<ImageType, ImageType>;
using RegistrationType = itk::ImageRegistrationMethodv4<ImageType, ImageType, TransformType>;
using OutputPixelType = unsigned char;
using OutputImageType = itk::Image<PixelType, Dimension>;
using CastFilterType = itk::CastImageFilter<ImageType, OutputImageType>;
using RescaleFilterType = itk::RescaleIntensityImageFilter<ImageType, OutputImageType>;
using WriterType = itk::ImageFileWriter<OutputImageType>;
using FixedImageReaderType = itk::ImageFileReader<ImageType>;


int main(int argc, char* argv[]){
    if(argc!=5){
        std::cout << "To run the executable you must provide four arguments";
    }

    std::string input_volume{argv[1]};
    std::string output_volume{argv[2]};
    std::string close_to_origin{argv[3]};
    std::string away_from_origin{argv[4]};

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
        std::cout << "must supply three coordinate values\n";
        return 1;
    }
    }

    std::vector<int> values_end;
    {
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
        std::cout << "must supply three coordinate values\n";
        return 1;
    }
    }

    for(size_t i = 0; i< 3; ++i)
        if(values_end[i] <= values_start[i]){
            std::printf("the (%d) ending coordinates (%d) must be larger than the start coordinates (%d)\n",i,values_end[i],values_start[i]);
            return 1;
        }
    
    
    auto fixedImageReader = FixedImageReaderType::New();
    fixedImageReader->SetFileName(input_volume);

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

    ImageType::IndexType start;
    start[0] = values_start[0];
    start[1] = values_start[1];
    start[2] = values_start[2];

    ImageType::SizeType end;
    end[0] = values_end[0];
    end[1] = values_end[1];
    end[2] = values_end[2];

    ImageType::RegionType region;
    region.SetIndex(start);
    region.SetSize(end);

    using FilterType = itk::RegionOfInterestImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(new_image);
    filter->SetRegionOfInterest(region);

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
    return 0;
}