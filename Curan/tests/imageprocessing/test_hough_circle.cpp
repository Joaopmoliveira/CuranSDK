#include "itkHoughTransform2DCirclesImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkThresholdImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkGradientMagnitudeImageFilter.h"
#include "itkDiscreteGaussianImageFilter.h"
#include <list>
#include "itkCastImageFilter.h"
#include "itkMath.h"


int main(int argc, char* argv[]) {
    // Create an image with a circle centered at [6, 8]:

    using ImageType = itk::Image<unsigned char, 2>;
    constexpr size_t width = 16;
    constexpr size_t height = 16;
    enum { centerX = 6, centerY = 8 };
    const auto image = ImageType::New();
    image->SetRegions({ width, height });
    image->Allocate(true);

    image->SetPixel({ centerX, centerY }, 1);
    image->SetPixel({ centerX, centerY - 1 }, 1);
    image->SetPixel({ centerX, centerY + 1 }, 1);
    image->SetPixel({ centerX - 1, centerY }, 1);
    image->SetPixel({ centerX + 1, centerY }, 1);

    using HoughTransformFilterType =
        itk::HoughTransform2DCirclesImageFilter<unsigned char,
        unsigned int,
        double>;

    const auto houghFilter = HoughTransformFilterType::New();
    houghFilter->SetInput(image);


    houghFilter->SetNumberOfCircles(1);
    houghFilter->SetMinimumRadius(1);
    houghFilter->SetMaximumRadius(1);
    houghFilter->Update();

    // GetCircles() finds a circle of radius 1, centered at [6, 8].
    const auto& spatialObject = houghFilter->GetCircles().front();
    spatialObject->ComputeObjectToWorldTransform();

    const double center[] = { centerX, centerY };
    const bool isInside = spatialObject->IsInside(center);

    std::cout << (isInside ?
        "OK: The center is inside." :
        "ERROR: The center is not inside!") << std::endl;

    auto circles = houghFilter->GetCircles();
    size_t iter = 1;
    for (const auto& circ : circles) {
        circ->ComputeObjectToWorldTransform();
        auto point = circ->GetCenterInObjectSpace();
        std::cout << "Circle : " << iter << " is at point : (" << point[0] << " , " << point[1] << ")\n";
        ++iter;
    }

    using OutputPixelType = unsigned char;
    using OutputImageType = itk::Image<OutputPixelType, 2>;
    OutputImageType::Pointer localOutputImage = OutputImageType::New();
    OutputImageType::RegionType region;
    region.SetSize(image->GetLargestPossibleRegion().GetSize());
    region.SetIndex(image->GetLargestPossibleRegion().GetIndex());
    localOutputImage->SetRegions(region);
    localOutputImage->SetOrigin(image->GetOrigin());
    localOutputImage->SetSpacing(image->GetSpacing());
    localOutputImage->Allocate(true);

    using CirclesListType = HoughTransformFilterType::CirclesListType;
    CirclesListType::const_iterator itCircles = circles.begin();
    ImageType::IndexType localIndex;
    while (itCircles != circles.end())
    {
        std::cout << "Center: ";
        std::cout << (*itCircles)->GetCenterInObjectSpace() << std::endl;
        std::cout << "Radius: " << (*itCircles)->GetRadiusInObjectSpace()[0]
            << std::endl;

        for (double angle = 0; angle <= itk::Math::twopi;
            angle += itk::Math::pi / 60.0)
        {
            const HoughTransformFilterType::CircleType::PointType centerPoint =
                (*itCircles)->GetCenterInObjectSpace();
            using IndexValueType = ImageType::IndexType::IndexValueType;
            localIndex[0] = itk::Math::Round<IndexValueType>(
                centerPoint[0] +
                (*itCircles)->GetRadiusInObjectSpace()[0] * std::cos(angle));
            localIndex[1] = itk::Math::Round<IndexValueType>(
                centerPoint[1] +
                (*itCircles)->GetRadiusInObjectSpace()[0] * std::sin(angle));
            OutputImageType::RegionType outputRegion =
                localOutputImage->GetLargestPossibleRegion();

            if (outputRegion.IsInside(localIndex))
            {
                localOutputImage->SetPixel(localIndex, 255);
            }
        }
        itCircles++;
    }


    std::cout << "Final:\n";
    ImageType::IndexType pixelIndex;
    for (long x = 0; x < width; ++x) {
        for (long y = 0; y < height; ++y) {
            pixelIndex = ImageType::IndexType{ y,x };
            std::cout << (double)localOutputImage->GetPixel(pixelIndex) << " , ";
        }
        std::cout << "\n";
    }

    return isInside ? EXIT_SUCCESS : EXIT_FAILURE;
}