#include "LoadVolume.h"
#include "itkShiftScaleImageFilter.h"

std::optional<ImageType::Pointer> get_volume(std::string path)
{
    using ReaderType = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderType::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();
    dicomIO->LoadSequencesOn();
    dicomIO->SetMaxSizeLoadEntry(0xffffff);
    dicomIO->SetKeepOriginalUID(true);
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

    std::string seriesIdentifier;
    seriesIdentifier = (seriesUID.begin())->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    try
    {
        reader->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }

    // ITK internally queries GDCM and obtains all the DICOM tags from the file
    // headers. The tag values are stored in the MetaDataDictionary
    // which is a general-purpose container for \{key,value\} pairs. The Metadata
    // dictionary can be recovered from any ImageIO class by invoking the
    // GetMetaDataDictionary() method.
    using DictionaryType = itk::MetaDataDictionary;

    const DictionaryType & dictionary = dicomIO->GetMetaDataDictionary();

    // In this example, we are only interested in the DICOM tags that can be
    // represented as strings. Therefore, we declare a MetaDataObject of
    // string type in order to receive those particular values.
    using MetaDataStringType = itk::MetaDataObject<std::string>;

    // The metadata dictionary is organized as a container with its corresponding
    // iterators. We can therefore visit all its entries by first getting access to
    // its Begin() and End() methods.
    auto itr = dictionary.Begin();
    auto end = dictionary.End();

    auto query = [&](const std::string& entryID){
        auto tagItr = dictionary.Find(entryID);
        std::optional<std::string> tagvalue = std::nullopt;
        if (tagItr == end){  
            std::cout << "Tag " << entryID;
            std::cout << " not found in the DICOM header" << std::endl;
            return tagvalue;
        }
        MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType *>(tagItr->second.GetPointer());

        if (entryvalue){
            tagvalue = entryvalue->GetMetaDataObjectValue();
            std::cout << "Patient's (" << entryID << ") ";
            std::cout << " is: " << *tagvalue << std::endl;
        }
        else
            std::cout << "Entry was not of string type" << std::endl;
        return tagvalue;
    };

    auto bits_stored_atribute = query("0028|0101");  //Bits Stored Attribute

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<DICOMImageType, DICOMImageType>;
    auto shiftFilter = ShiftScaleFilterType::New();
    
    auto compute_conversion = [](const std::string& number_in_string){
        std::optional<int> result = std::nullopt;
        int result_l{};
        auto [ptr, ec] = std::from_chars(number_in_string.data(), number_in_string.data() + number_in_string.size(), result_l);
 
        if (ec == std::errc())
            result = result_l;
        
        return result;
    };


    int bits_stored = bits_stored_atribute ? (compute_conversion(*bits_stored_atribute) ?  *compute_conversion(*bits_stored_atribute) : 16) : 16;  //Bits Allocated Attribute

    std::printf("scalling factor is: (%f) with bits stored (%d)\n",std::pow(2,sizeof(PixelType)*8.0-bits_stored+1),bits_stored);
    
    shiftFilter->SetScale(std::pow(2,sizeof(PixelType)*8.0-bits_stored+1));
    shiftFilter->SetShift(0);
    shiftFilter->SetInput(reader->GetOutput());

    using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(shiftFilter->GetOutput());

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