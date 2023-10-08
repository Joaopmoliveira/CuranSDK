#ifndef LINK_REGISTRATION_ROBOT_HEADER
#define LINK_REGISTRATION_ROBOT_HEADER

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
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include <iostream>
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "utils/TheadPool.h"
#include "rendering/Volume.h"
#include <tuple>

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

void updateBaseTexture3D(vsg::floatArray3D &image, ImageType::Pointer image_to_render);

class CommandType : public itk::Command
{
public:
    using Self = CommandType;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro(Self);

protected:
    CommandType() = default;

public:
    using OptimizerType = itk::RegularStepGradientDescentOptimizerv4<double>;
    using OptimizerPointer = const OptimizerType *;
    curan::renderable::Volume *moving_pointer_to_volume = nullptr;
    Eigen::Matrix<double, 4, 4> moving_homogenenous;
    TransformType::Pointer initialTransform;

    void set_pointer(curan::renderable::Volume *in_moving_pointer_to_volume);

    void Execute(itk::Object *caller, const itk::EventObject &event) override;

    void Execute(const itk::Object *object, const itk::EventObject &event) override;
};

struct CurrentInitialPose {
    Eigen::Matrix<double, 4, 4> mat_moving_here = Eigen::Matrix<double, 4, 4>::Identity();
    std::mutex mut;

    void update_matrix(const Eigen::Matrix<double, 4, 4>& supply);

    Eigen::Matrix<double,4,4> get_matrix();
};

struct info_solve_registration{
    ImageType::Pointer fixed_image;
    ImageType::Pointer moving_image;
    curan::renderable::Volume *volume_moving;
    CurrentInitialPose& moving_homogenenous;
    std::atomic<bool>& optimization_running;
    std::atomic<bool>& robot_client_commands_volume_init;
    std::shared_ptr<curan::utilities::ThreadPool> thread_pool;
    std::vector<std::tuple<double,TransformType::Pointer>>& full_runs;
    std::unique_ptr<kuka::Robot> robot; 
    std::unique_ptr<RobotParameters> iiwa;
    vsg::ref_ptr<curan::renderable::Renderable> robot_render;
};

std::tuple<double, TransformType::Pointer> solve_registration(info_solve_registration &info_registration);

#endif