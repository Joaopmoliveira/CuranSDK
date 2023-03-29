#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImportImageFilter.h"
#include "itkCannyEdgeDetectionImageFilter.h"

int main() {
    constexpr long width = 5;
    constexpr long height = 5;
    constexpr unsigned int Dimension = 2;
    using CharPixelType = unsigned char; //  IO
    using RealPixelType = double;        //  Operations

    using CharImageType = itk::Image<CharPixelType, Dimension>;
    using RealImageType = itk::Image<RealPixelType, Dimension>;

    using ImageType = itk::Image< CharPixelType, Dimension >;
    using RealType = itk::Image< RealImageType, Dimension >;
    using ImportFilterType = itk::ImportImageFilter< CharPixelType, Dimension >;

    using CastToRealFilterType =
        itk::CastImageFilter<CharImageType, RealImageType>;
    using CannyFilterType =
        itk::CannyEdgeDetectionImageFilter<RealImageType, RealImageType>;
    using RescaleFilterType =
        itk::RescaleIntensityImageFilter<RealImageType, CharImageType>;

    ImportFilterType::SizeType size;
    size[0] = width; // size along X
    size[1] = height; // size along Y
    ImportFilterType::IndexType start;
    start.Fill(0);

    unsigned char localBuffer[width * height];

    std::cout << "Original:\n";
    for (long y = 0; y < height; ++y) {
        for (long x = 0; x < width; ++x) {
            localBuffer[x + y * width] = y;
            std::cout << (int)localBuffer[x + y*width] << " , ";
        }
        std::cout << "\n";
    }

    const bool importImageFilterWillOwnTheBuffer = false;

    auto importFilter = ImportFilterType::New();

    ImportFilterType::RegionType region;
    region.SetIndex(start);
    region.SetSize(size);
    importFilter->SetRegion(region);
    itk::SpacePrecisionType origin[2];
    origin[0] = 0;
    origin[1] = 0;
    importFilter->SetOrigin(origin);
    itk::SpacePrecisionType spacing[2];
    spacing[0] = 1,0;
    spacing[1] = 1.0;
    importFilter->SetImportPointer(&localBuffer[0], width * height,
        importImageFilterWillOwnTheBuffer);

    try
    {
        importFilter->Update();
    }
    catch (const itk::ExceptionObject& err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    auto out = importFilter->GetOutput();
    std::cout << "Import filter:\n";
    ImageType::IndexType pixelIndex;
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = ImageType::IndexType{ y,x };
            std::cout << (double)out->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }

    auto toReal = CastToRealFilterType::New();
    auto cannyFilter = CannyFilterType::New();

    auto rescale = RescaleFilterType::New();

    toReal->SetInput(out);

    try
    {
        toReal->Update();
    }
    catch (const itk::ExceptionObject& err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    auto val = toReal->GetOutput();
    std::cout << "To real filter:\n";
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = ImageType::IndexType{ y,x };
            std::cout << (double)val->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }

    double variance = 1;
    double upperThreshold = 0;
    double lowerThreshold = 100;

    cannyFilter->SetVariance(variance);
    cannyFilter->SetUpperThreshold(upperThreshold);
    cannyFilter->SetLowerThreshold(lowerThreshold);
    cannyFilter->SetInput(toReal->GetOutput());
    try
    {
        cannyFilter->Update();
    }
    catch (const itk::ExceptionObject& err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Canny filter:\n";
    auto out1 = cannyFilter->GetOutput();
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = ImageType::IndexType{ y,x };
            std::cout << (double)out1->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }

    rescale->SetInput(cannyFilter->GetOutput());

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject& err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Rescale filter:\n";
    auto out2 = cannyFilter->GetOutput();
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = ImageType::IndexType{ y,x };
            std::cout << (double)out2->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }

    return EXIT_SUCCESS;
}