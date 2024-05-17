#include "itkMultiResolutionImageRegistrationMethod.h"
#include "itkVersorRigid3DTransform.h"
#include "itkCenteredTransformInitializer.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkVersorRigid3DTransformOptimizer.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCheckerBoardImageFilter.h"
#include "itkCommand.h"
#include "itkShiftScaleImageFilter.h"
#include "itkImageSeriesReader.h"
#include "itkGDCMImageIO.h"
#include "itkImageDuplicator.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkCompositeTransform.h"
#include "itkSubtractImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"

#include <optional>
#include <charconv>

template <typename TRegistration>
class RegistrationInterfaceCommand : public itk::Command
{
public:
    using Self = RegistrationInterfaceCommand;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    RegistrationInterfaceCommand(){};

public:
    using RegistrationType = TRegistration;
    using RegistrationPointer = RegistrationType *;
    using OptimizerType = itk::VersorRigid3DTransformOptimizer;
    using OptimizerPointer = OptimizerType *;
    void Execute(itk::Object *object, const itk::EventObject &event)
    {
        if (!(itk::IterationEvent().CheckEvent(&event)))
        {
            return;
        }
        RegistrationPointer registration = static_cast<RegistrationPointer>(object);
        OptimizerPointer optimizer = static_cast<OptimizerPointer>(registration->GetModifiableOptimizer());
        std::cout << "-------------------------------------" << std::endl;
        std::cout << "MultiResolution Level : "
                   << registration->GetCurrentLevel() << std::endl;
        std::cout << std::endl;
        if (registration->GetCurrentLevel() == 0)
        {
            optimizer->SetMaximumStepLength(1.00);
            optimizer->SetMinimumStepLength(0.01);
        }
        else
        {
            optimizer->SetMaximumStepLength(
                optimizer->GetMaximumStepLength() * 0.35);
            optimizer->SetMinimumStepLength(
                optimizer->GetMinimumStepLength() * 0.3);
        }
    }
    void Execute(const itk::Object *, const itk::EventObject &)
    {
        return;
    }
};

class CommandIterationUpdate : public itk::Command
{
public:
    using Self = CommandIterationUpdate;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandIterationUpdate(){};

public:
    using OptimizerType = itk::VersorRigid3DTransformOptimizer;
    using OptimizerPointer = const OptimizerType *;
    void Execute(itk::Object *caller, const itk::EventObject &event) override
    {
        Execute((const itk::Object *)caller, event);
    }
    void Execute(const itk::Object *object, const itk::EventObject &event) override
    {
        OptimizerPointer optimizer = static_cast<OptimizerPointer>(object);
        if (!(itk::IterationEvent().CheckEvent(&event)))
        {
            return;
        }
        //std::cout << optimizer->GetCurrentIteration() << "\n";
        //std::cout << optimizer->GetValue() << "\n";
    }
};

constexpr unsigned int Dimension = 3;
using PixelType = uint16_t;

using FixedImageType = itk::Image<PixelType, Dimension>;
using MovingImageType = itk::Image<PixelType, Dimension>;

int main(int argc, const char *argv[])
{
    try
    {
        if(argc!=4){
            if(argc>4 || argc == 1){
                std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - input volume, (fixed)\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
            }
            if(argc == 2){
                std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - input volume (static)\n"
                      << "third parameter - output volume";
                      return 1;
            }
            if(argc == 3){
                std::cout << "To run the executable you must provide three arguments:\n "
                      << "first parameter - " << std::string(argv[1]) << "\n"
                      << "second parameter - "<< std::string(argv[2]) << "\n"
                      << "third parameter - output volume";
                      return 1;
            }
        }

        std::string input_volume{argv[1]};
        std::string moved_input_volume{argv[2]};
        std::string output_volume{argv[3]};

        using FixedImageReaderType = itk::ImageFileReader<FixedImageType>;
        auto fixedImageReader = FixedImageReaderType::New();
        fixedImageReader->SetFileName(input_volume);

        using MovingImageReaderType = itk::ImageFileReader<MovingImageType>;
        auto movingImageReader = MovingImageReaderType::New();
        movingImageReader->SetFileName(moved_input_volume);

        try
        {
            fixedImageReader->Update();
            movingImageReader->Update();
        }
        catch (itk::ExceptionObject &err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            return EXIT_FAILURE;
        }

        FixedImageType::Pointer image_to_register = fixedImageReader->GetOutput();;
        MovingImageType::Pointer moving_image_to_register = movingImageReader->GetOutput();


        auto print_image_info = [](itk::Image<PixelType,3>::Pointer image, std::string name){
            std::cout << "(" << name << ") : -------------------\ndirection :\n";
            auto direction = image->GetDirection();
            for(size_t i = 0; i < 3; ++i){
                for(size_t j = 0; j < 3; ++j)
                    std::cout <<  direction(i,j) << " ";
                std::cout << "\n";
            }
            auto origin = image->GetOrigin();
            std::cout << "\norigin :";
            for(size_t i = 0; i < 3; ++i){
                std::cout <<  origin[i] << " ";
            }
            std::cout << "\n-------------------\n";
        };

        print_image_info(image_to_register,"fixed image");
        print_image_info(image_to_register,"moving image");

        using InternalPixelType = float;
        using InternalImageType = itk::Image<InternalPixelType, Dimension>;
        using TransformType = itk::VersorRigid3DTransform<double>;
        using OptimizerType = itk::VersorRigid3DTransformOptimizer;
        using InterpolatorType = itk::LinearInterpolateImageFunction<
            InternalImageType,
            double>;
        using MetricType = itk::MattesMutualInformationImageToImageMetric<
            InternalImageType,
            InternalImageType>;
        using RegistrationType = itk::MultiResolutionImageRegistrationMethod<
            InternalImageType,
            InternalImageType>;
        using FixedImagePyramidType = itk::MultiResolutionPyramidImageFilter<
            InternalImageType, InternalImageType>;
        using MovingImagePyramidType = itk::MultiResolutionPyramidImageFilter<
            InternalImageType, InternalImageType>;
        TransformType::Pointer transform = TransformType::New();

        

        using TransformInitializerType = itk::CenteredTransformInitializer<
            TransformType,
            FixedImageType, MovingImageType>;
        TransformInitializerType::Pointer initializer = TransformInitializerType::New();

        initializer->SetTransform(transform);
        initializer->SetFixedImage(image_to_register);
        initializer->SetMovingImage(moving_image_to_register);

        initializer->MomentsOn();
        initializer->InitializeTransform();
        auto parameters = transform->GetParameters();

        // we randomize the orientation information
        parameters[0] += 1;
        parameters[1] += 1;
        parameters[2] += 1;

        transform->SetParameters(parameters);

        assert(transform->GetNumberOfParameters() == 6);
        OptimizerType::Pointer optimizer = OptimizerType::New();
        OptimizerType::ScalesType optimizerScales(6);
        optimizerScales[0] = 1.0;
        optimizerScales[1] = 1.0;
        optimizerScales[2] = 1.0;
        optimizerScales[3] = 1.0 / 1000.0;
        optimizerScales[4] = 1.0 / 1000.0;
        optimizerScales[5] = 1.0 / 1000.0;
        optimizer->SetScales(optimizerScales);
        InterpolatorType::Pointer interpolator = InterpolatorType::New();
        RegistrationType::Pointer registration = RegistrationType::New();

        MetricType::Pointer metric = MetricType::New();
        FixedImagePyramidType::Pointer fixedImagePyramid =
            FixedImagePyramidType::New();
        MovingImagePyramidType::Pointer movingImagePyramid =
            MovingImagePyramidType::New();

        registration->SetInitialTransformParameters(transform->GetParameters());

        registration->SetOptimizer(optimizer);
        registration->SetTransform(transform);
        registration->SetInterpolator(interpolator);
        registration->SetMetric(metric);
        registration->SetFixedImagePyramid(fixedImagePyramid);
        registration->SetMovingImagePyramid(movingImagePyramid);

        using FixedCastFilterType = itk::CastImageFilter<FixedImageType, InternalImageType>;
        using MovingCastFilterType = itk::CastImageFilter<MovingImageType, InternalImageType>;

        FixedCastFilterType::Pointer fixedCaster = FixedCastFilterType::New();
        MovingCastFilterType::Pointer movingCaster = MovingCastFilterType::New();

        fixedCaster->SetInput(image_to_register);
        movingCaster->SetInput(moving_image_to_register);
        registration->SetFixedImage(fixedCaster->GetOutput());
        registration->SetMovingImage(movingCaster->GetOutput());
        fixedCaster->Update();
        registration->SetFixedImageRegion(fixedCaster->GetOutput()->GetBufferedRegion());
        using ParametersType = RegistrationType::ParametersType;
        metric->SetNumberOfHistogramBins(50);
        metric->SetNumberOfSpatialSamples(0.2*image_to_register->GetBufferedRegion().GetSize()[0]*image_to_register->GetBufferedRegion().GetSize()[1]*image_to_register->GetBufferedRegion().GetSize()[2]);
        
        metric->SetNumberOfHistogramBins(10);
        metric->ReinitializeSeed(1);
        metric->SetUseExplicitPDFDerivatives(true);

        optimizer->SetNumberOfIterations(400);
        optimizer->SetRelaxationFactor(0.8);

        CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
        optimizer->AddObserver(itk::IterationEvent(), observer);
        using CommandType = RegistrationInterfaceCommand<RegistrationType>;
        CommandType::Pointer command = CommandType::New();
        registration->AddObserver(itk::IterationEvent(), command);
        registration->SetNumberOfLevels(4);

        constexpr unsigned int numberOfLevels = 4;

        registration->SetNumberOfLevels(numberOfLevels);

        try
        {
            registration->Update();
            std::cout << "Optimizer stop condition: "
                      << registration->GetOptimizer()->GetStopConditionDescription()
                      << std::endl;
        }
        catch (itk::ExceptionObject &err)
        {
            std::cout << "ExceptionObject caught !" << std::endl;
            std::cout << err << std::endl;
            return EXIT_FAILURE;
        }

        ParametersType finalParameters = registration->GetLastTransformParameters();
        ParametersType finalParameters2 = registration->GetOutput()->Get()->GetParameters();

        unsigned int numberOfIterations = optimizer->GetCurrentIteration();
        double bestValue = optimizer->GetValue();

        std::printf("\n============\nTranslation (%f %f %f) \nRotation (%f %f %f) \nIterations (%d) \nBest Value (%f)\n", 
                                finalParameters[3], 
                                finalParameters[4], 
                                finalParameters[5], 
                                finalParameters[0], 
                                finalParameters[1], 
                                finalParameters[2], 
                                numberOfIterations, 
                                bestValue);

        std::printf("\n============\nTranslation (%f %f %f) \nRotation (%f %f %f) \nIterations (%d) \nBest Value (%f)\n", 
                                finalParameters2[3], 
                                finalParameters2[4], 
                                finalParameters2[5], 
                                finalParameters2[0], 
                                finalParameters2[1], 
                                finalParameters2[2], 
                                numberOfIterations, 
                                bestValue);

        using ResampleFilterType = itk::ResampleImageFilter<
            InternalImageType,
            InternalImageType>;
        TransformType::Pointer finalTransform = TransformType::New();
        finalTransform->SetParameters(finalParameters);
        finalTransform->SetFixedParameters(transform->GetFixedParameters());

        {
            using ResampleFilterType =
                itk::ResampleImageFilter<MovingImageType, FixedImageType>;
 
            auto resample = ResampleFilterType::New();
            resample->SetTransform(registration->GetTransform());
            resample->SetInput(movingImageReader->GetOutput());
 
            FixedImageType::Pointer fixedImage = fixedImageReader->GetOutput();
 
            resample->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
            resample->SetOutputOrigin(fixedImage->GetOrigin());
            resample->SetOutputSpacing(fixedImage->GetSpacing());
            resample->SetOutputDirection(fixedImage->GetDirection());
            resample->SetDefaultPixelValue(0);
 
 
             using OutputPixelType = unsigned char;
 
            using OutputImageType = itk::Image<OutputPixelType, Dimension>;
 
            using CastFilterType =
                itk::CastImageFilter<FixedImageType, OutputImageType>;
 
            using WriterType = itk::ImageFileWriter<OutputImageType>;
 
             auto writer = WriterType::New();
            auto caster = CastFilterType::New();
 
            writer->SetFileName(output_volume+".mha");
 
            caster->SetInput(resample->GetOutput());
            writer->SetInput(caster->GetOutput());
            writer->Update();
        }

        return EXIT_SUCCESS;
    }
    catch (const itk::ExceptionObject &excp)
    {
        std::cerr << "Problem solving registration= " << std::endl;
        std::cerr << argv[0] << std::endl;
        std::cerr << excp << std::endl;
        return EXIT_FAILURE;
    }
}
