#include "DicomLoading.h"

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