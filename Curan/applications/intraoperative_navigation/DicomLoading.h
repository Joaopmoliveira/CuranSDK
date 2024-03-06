#include "itkImage.h"
#include "itkImageFileReader.h"

#include "itkResampleImageFilter.h"

#include "itkEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkIntensityWindowingImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkMinimumMaximumImageCalculator.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"

#include <optional>
#include <chrono>
#include <thread>
#include "Mathematics/IntrPlane3OrientedBox3.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"

constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;
using DicomPixelType = unsigned short;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension_in>;

using InterImageType = itk::Image<InterPixelType, Dimension_out>;

// using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;

using ReaderType = itk::ImageFileReader<InputImageType>;

InputImageType::Pointer load_dicom()
{

    using ReaderTypeDicom = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderTypeDicom::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524"};
    nameGenerator->SetDirectory(dirName_input);

    using SeriesIdContainer = std::vector<std::string>;

    const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();

    auto seriesItr = seriesUID.begin();
    auto seriesEnd = seriesUID.end();
    while (seriesItr != seriesEnd)
    {
        std::cout << seriesItr->c_str() << std::endl;
        ++seriesItr;
    }

    std::string seriesIdentifier;
    seriesIdentifier = seriesUID.begin()->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(reader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<InputPixelType>::max());

    using FilterType = itk::CastImageFilter<DICOMImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return nullptr;
    }

    using ImageCalculatorFilterType = itk::MinimumMaximumImageCalculator<InputImageType>;

    auto imageCalculatorFilter = ImageCalculatorFilterType::New();
    imageCalculatorFilter->SetImage(filter->GetOutput());
    imageCalculatorFilter->Compute();

    imageCalculatorFilter->GetMaximum();
    imageCalculatorFilter->GetMinimum();
    using IntensityFilterType = itk::IntensityWindowingImageFilter<InputImageType, InputImageType>;

    auto intensityWindowing = IntensityFilterType::New();
    intensityWindowing->SetWindowMinimum(0.0);
    intensityWindowing->SetWindowMaximum(100);

    intensityWindowing->SetOutputMinimum(0.0);
    intensityWindowing->SetOutputMaximum(itk::NumericTraits<InputPixelType>::max());

    intensityWindowing->SetInput(filter->GetOutput());

    try
    {
        intensityWindowing->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return nullptr;
    }

    return intensityWindowing->GetOutput();
};