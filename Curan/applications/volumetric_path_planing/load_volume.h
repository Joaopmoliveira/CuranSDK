#ifndef LOAD_VOLUME_HEADER
#define LOAD_VOLUME_HEADER

#include "common_includes.h"

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

    while (itr != end)
    {
        itk::MetaDataObjectBase::Pointer entry = itr->second;
 
        MetaDataStringType::Pointer entryvalue =
            dynamic_cast<MetaDataStringType *>(entry.GetPointer());
    
        if (entryvalue)
        {
            std::string tagkey = itr->first;
            std::string labelId;
            bool found = itk::GDCMImageIO::GetLabelFromTag(tagkey, labelId);
      
            std::string tagvalue = entryvalue->GetMetaDataObjectValue();

            if (found)
            {
                std::cout << "(" << tagkey << ") " << labelId;
                std::cout << " = " << tagvalue.c_str() << std::endl;
            } else {
                std::cout << "(" << tagkey << ") "
                  << "Unknown";
                std::cout << " = " << tagvalue.c_str() << std::endl;
            }
    }

    auto query = [&](const std::string& entryID){
        auto tagItr = dictionary.Find(entryID);

        if (tagItr == end)
        {
            std::cerr << "Tag " << entryID;
            std::cerr << " not found in the DICOM header" << std::endl;
        } else {
            // Since the entry may or may not be of string type we must again use a
            // dynamic_cast in order to attempt to convert it to a string dictionary
            // entry. If the conversion is successful, we can then print out its content.
            MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType *>(tagItr->second.GetPointer());

            if (entryvalue){
                std::string tagvalue = entryvalue->GetMetaDataObjectValue();
                std::cout << "Patient's (" << entryID << ") ";
                std::cout << " is: " << tagvalue << std::endl;
            }
            else
                std::cerr << "Entry was not of string type" << std::endl;
        }
    };

    query("0028,0002");
    query("0028,0101");
    query("0028,0107");
    query("0028,2000");         
    query("0028,2002"); 

    return filter->GetOutput();
}

#endif