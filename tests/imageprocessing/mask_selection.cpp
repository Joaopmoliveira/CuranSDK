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
using MovingImageReaderType = itk::ImageFileReader<ImageType>;

int main(){
    auto fixedImageReader = FixedImageReaderType::New();


    std::string dirName{CURAN_COPIED_RESOURCE_PATH "/original_volume.mha"};
    fixedImageReader->SetFileName(dirName);


    try
    {
        fixedImageReader->Update();

    }
    catch (...)
    {
        std::cout << "Failed to read the Moving and Fixed images\nplease make sure that you have properly added them to the path:\n" << std::string(CURAN_COPIED_RESOURCE_PATH);
        return 1;
    }

    ImageType::Pointer pointer2fixedimage = fixedImageReader->GetOutput();


    std::vector<std::array<double, 6>> internals;
    internals.push_back({});

    auto evaluate_if_pixel_inside_mask = [&](double in_x, double in_y, double in_z)
        {
            for (const auto &boundary : internals)
            {
                if (in_x > boundary[0] && in_x < boundary[3] && in_y > boundary[1] && in_y < boundary[4] && in_z > boundary[2] && in_z < boundary[5])
                    return true;
            }
            return false;
        };

        using IteratorType = itk::ImageRegionIteratorWithIndex<ImageType>;
        IteratorType outputIt(masked_output_image, masked_output_image->GetRequestedRegion());

        for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
        {
            ImageType::IndexType idx = outputIt.GetIndex();
            if (!evaluate_if_pixel_inside_mask(idx[0], idx[1], idx[2]))
                outputIt.Set(0);
        }

        using WriterType = itk::ImageFileWriter<ImageType>;

        {
            auto writer = WriterType::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/original_volume.mha");
            writer->SetInput(data_application.map[ORIGINAL_VOLUME].get_volume());
            writer->Update();
        }

        {
            auto writer = WriterType::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/masked_volume.mha");
            writer->SetInput(masked_output_image);
            writer->Update();
        }

    return 0;
}