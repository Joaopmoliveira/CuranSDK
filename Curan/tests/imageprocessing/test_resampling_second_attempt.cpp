#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkImageFileWriter.h"
#include "itkCenteredEuler3DTransform.h"

#include <optional>

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;
using WriterType = itk::ImageFileWriter<ImageType>;

std::optional<ImageType::Pointer> get_volume(std::string path)
{
    using ReaderType = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderType::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    nameGenerator->SetDirectory(path);

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
    rescale->SetOutputMaximum(itk::NumericTraits<PixelType>::max());

    using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }

    return filter->GetOutput();
}

int main()
{
    auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524");
    if (!volume)
        return 1;
    
    auto input = *volume;

    using InputWriterType = itk::ImageFileWriter<ImageType>;
    auto input_writer = InputWriterType::New();

    input_writer->SetFileName("input_my_test.mha");
    input_writer->SetInput(input);
    try
    {
        input_writer->Update();
    }
    catch (...){
        std::cout << "failure\n";
        return 1;
    }

    auto size = input->GetLargestPossibleRegion().GetSize();
    ImageType::IndexType index{size[0]/2.0,size[1]/2.0,size[2]/2.0};
    ImageType::PointType center;
    input->TransformIndexToPhysicalPoint(index,center);

    using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();

    using TransformType = itk::IdentityTransform<double>;
    auto transform = TransformType::New();

    Eigen::AngleAxis<double> relative_transformation{45.0*3.1415/180.0,Eigen::Vector<double,3>{1.0,0.0,0.0}};
    auto matrix = relative_transformation.matrix();

    itk::Matrix<double> rotation;
    rotation.SetIdentity();
    for(size_t row = 0; row < 3; ++row)
        for(size_t col = 0; col < 3; ++col)
            rotation(row,col) = matrix(row,col);

    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(70);
    filter->SetTransform(transform);

    auto spacing = input->GetSpacing();
    auto output_size = size;
    auto output_spacing = spacing;
    double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);
    size_t maximum_size = std::max(std::max(size[0],size[1]),size[2]);

    for (size_t row = 0; row < 3; ++row){
        output_spacing[row] = minimum_spacing;
        output_size[row] = maximum_size;
    }

    filter->SetInput(input);
    filter->SetOutputSpacing(output_spacing);
    filter->SetSize(output_size);

    using WriterType = itk::ImageFileWriter<ImageType>;
    auto writer = WriterType::New();

    writer->SetFileName("my_test.mha");
    writer->SetInput(filter->GetOutput());

    try
    {
        writer->Update();
    }
    catch (...){
        std::cout << "failure\n";
        return 1;
    }

    return 0;
}