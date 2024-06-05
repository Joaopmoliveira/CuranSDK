#include <iostream>
#include <string>
#include <sstream>
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkThresholdImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkCastImageFilter.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkRelabelComponentImageFilter.h"
#include "itkMaskImageFilter.h"

using PixelType = float;
using ImageType = itk::Image<PixelType, 3>;

ImageType::Pointer PreProcessImage(ImageType::Pointer volume) {
    using ThresholdFilterType = itk::ThresholdImageFilter<ImageType>;
    ThresholdFilterType::Pointer thresholdFilter = ThresholdFilterType::New();
    thresholdFilter->SetInput(volume);
    thresholdFilter->ThresholdOutside(0.5, 1);
    thresholdFilter->SetOutsideValue(0);
    thresholdFilter->Update();

    ImageType::Pointer thresholdedVolume = thresholdFilter->GetOutput();
    std::cout << "Thresholding completed." << std::endl;

    using LabelType = unsigned short;
    using LabelImageType = itk::Image<LabelType, 3>;
    using ConnectedComponentFilterType = itk::ConnectedComponentImageFilter<ImageType, LabelImageType>;
    ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();
    connectedComponentFilter->SetInput(thresholdedVolume);
    connectedComponentFilter->Update();

    std::cout << "Connected components filter completed." << std::endl;

    using RelabelFilterType = itk::RelabelComponentImageFilter<LabelImageType, LabelImageType>;
    RelabelFilterType::Pointer relabelFilter = RelabelFilterType::New();
    relabelFilter->SetInput(connectedComponentFilter->GetOutput());
    relabelFilter->Update();

    std::cout << "Relabeling completed. Number of objects: " << relabelFilter->GetNumberOfObjects() << std::endl;

    using ThresholdFilterType2 = itk::ThresholdImageFilter<LabelImageType>;
    ThresholdFilterType2::Pointer thresholdFilter2 = ThresholdFilterType2::New();
    thresholdFilter2->SetInput(relabelFilter->GetOutput());
    thresholdFilter2->ThresholdOutside(1, 1); // Keep only the label 1 (largest component)
    thresholdFilter2->SetOutsideValue(0);
    thresholdFilter2->Update();

    LabelImageType::Pointer largestComponentMask = thresholdFilter2->GetOutput();
    std::cout << "Thresholding to keep largest component completed." << std::endl;

    using MaskFilterType = itk::MaskImageFilter<ImageType, LabelImageType, ImageType>;
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    maskFilter->SetInput(volume);
    maskFilter->SetMaskImage(largestComponentMask);
    maskFilter->Update();

    return maskFilter->GetOutput();
}

int write_volume(ImageType::Pointer volume, const std::string& path) {
    auto writer = itk::ImageFileWriter<ImageType>::New();
    writer->SetFileName(path);
    writer->SetInput(volume);

    try {
        writer->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Volume written to " << path << std::endl;
    return EXIT_SUCCESS;
}

ImageType::Pointer read_volume(const std::string& image_path) {
    using ImageReaderType = itk::ImageFileReader<ImageType>;

    auto imageReader = ImageReaderType::New();
    imageReader->SetFileName(image_path);

    try {
        imageReader->Update();
    } catch (const itk::ExceptionObject & error) {
        std::cerr << "Error: " << error << std::endl;
        return nullptr;
    }

    using MinMaxCalculatorType = itk::MinimumMaximumImageCalculator<ImageType>;
    auto minMaxCalculator = MinMaxCalculatorType::New();
    minMaxCalculator->SetImage(imageReader->GetOutput());
    minMaxCalculator->Compute();

    PixelType minValue = minMaxCalculator->GetMinimum();
    PixelType maxValue = minMaxCalculator->GetMaximum();

    std::cout << "Minimum pixel value: " << minValue << std::endl;
    std::cout << "Maximum pixel value: " << maxValue << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    return imageReader->GetOutput();
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <ImageFile>" << std::endl;
        std::cerr << "The files must be in the post build resource path, in the precious phantom folder." << std::endl;
        return EXIT_FAILURE;
    }

    std::string aux = "/precious_phantom/";
    std::string image{argv[1]};
    std::string image_path = CURAN_COPIED_RESOURCE_PATH + aux + image;

    std::string output = "filtered_volume.mha";
    std::string output_path = CURAN_COPIED_RESOURCE_PATH + aux + output;

    auto volume = read_volume(image_path);
    if (!volume) {
        std::cerr << "Error: Failed to read volume." << std::endl;
        return EXIT_FAILURE;
    }

    auto volume_filtered = PreProcessImage(volume);
    if (!volume_filtered) {
        std::cerr << "Error: Preprocessing failed." << std::endl;
        return EXIT_FAILURE;
    }

    if (write_volume(volume_filtered, output_path) != EXIT_SUCCESS) {
        std::cerr << "Error: Failed to write volume." << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
