#include "itkImage.h"
#include "itkImportImageFilter.h"
#include "itkImageFileWriter.h"

#include "imageprocessing/FilterAlgorithms.h"

template <typename TImage>
void DeepCopy(typename TImage::Pointer input, typename TImage::Pointer output)
{
    output->SetRegions(input->GetLargestPossibleRegion());
    output->Allocate();

    itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage>      outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd())
    {
        outputIterator.Set(inputIterator.Get());
        ++inputIterator;
        ++outputIterator;
    }
}

void test_binarization() {
    constexpr long width = 5;
    constexpr long height = 5;

    curan::image::filtering::ImportFilter::ImportFilterType::SizeType size;
    size[0] = width; // size along X
    size[1] = height; // size along Y
    curan::image::filtering::ImportFilter::ImportFilterType::IndexType start;
    start.Fill(0);

    unsigned char localBuffer[width * height] = { 125 , 125 , 125 , 125 , 125 ,
                                                 136 , 136 , 136 , 136 , 136 ,
                                                 150 , 150 , 150 , 150 , 150 ,
                                                 200 , 200 , 200 , 200 , 200 ,
                                                 222 , 222 , 222 , 222 , 222 };

    const bool importImageFilterWillOwnTheBuffer = false;

    curan::image::filtering::ImportFilter::Info info_intro;
    info_intro.buffer = localBuffer;
    info_intro.memory_owner = false;
    info_intro.number_of_pixels = width * height;
    info_intro.origin = { 0.0, 0.0 };
    info_intro.size = size;
    info_intro.spacing = { 1.0, 1.0 };
    info_intro.start = start;
    auto import_filer = curan::image::filtering::ImportFilter::make(info_intro);

    curan::image::filtering::BinarizeFilter::Info info;
    info.outside_value = 0;
    info.inside_value = 100;
    info.higher_value = 199;
    info.lower_value = 137;
    auto binarizer = curan::image::filtering::BinarizeFilter::make(info);

    curan::image::filtering::Filter filter;
    filter << import_filer;
    filter << binarizer;

    auto filtered_output = filter.get_output();

    std::cout << "Original image:\n 125 , 125 , 125 , 125 , 125 ,\n 136, 136, 136, 136, 136,\n 150, 150, 150, 150, 150,\n 200, 200, 200, 200, 200,\n 222, 222, 222, 222, 222\n";

    std::cout << "Expected image:\n 0 , 0 , 0 , 0 , 0 ,\n 0 , 0 , 0 , 0 , 0 ,\n 100, 100, 100, 100, 100,\n 0 , 0 , 0 , 0 , 0 ,\n 0 , 0 , 0 , 0 , 0\n";

    std::cout << "Real image: \n";
    curan::image::Internal2DImageType::IndexType pixelIndex;
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = curan::image::Internal2DImageType::IndexType{ y,x };
            std::cout << (int)filtered_output->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }
}

void test_canny() {
    constexpr long width = 10;
    constexpr long height = 10;

    curan::image::filtering::ImportFilter::ImportFilterType::SizeType size;
    size[0] = width; // size along X
    size[1] = height; // size along Y
    curan::image::filtering::ImportFilter::ImportFilterType::IndexType start;
    start.Fill(0);

    unsigned char localBuffer[width * height];

    std::cout << "Original:\n";
    for (long y = 0; y < height; ++y) {
        for (long x = 0; x < width; ++x) {
            localBuffer[x + y * width] = y;
            std::cout << (int)localBuffer[x + y * width] << " , ";
        }
        std::cout << "\n";
    }

    curan::image::filtering::ImportFilter::Info info_intro;
    info_intro.buffer = localBuffer;
    info_intro.memory_owner = false;
    info_intro.number_of_pixels = width * height;
    info_intro.origin = { 0.0, 0.0 };
    info_intro.size = size;
    info_intro.spacing = { 1.0, 1.0 };
    info_intro.start = start;
    auto import_filer = curan::image::filtering::ImportFilter::make(info_intro);

    curan::image::filtering::CannyFilter::Info info;
    info.lower_bound = 100;
    info.upper_bound = 0;
    info.variance = 1;
    auto canny = curan::image::filtering::CannyFilter::make(info);

    curan::image::filtering::Filter filter;
    filter << import_filer;
    filter << canny;

    auto filtered_output = filter.get_output();

    std::cout << "Real image: \n";
    curan::image::Internal2DImageType::IndexType pixelIndex;
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = curan::image::Internal2DImageType::IndexType{ y,x };
            std::cout << (int)filtered_output->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }
}

int main() {
    test_binarization();
    test_canny();
	return 0;
}
