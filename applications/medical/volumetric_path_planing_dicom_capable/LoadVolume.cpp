#include "LoadVolume.h"
#include "itkShiftScaleImageFilter.h"
#include "itkOrientImageFilter.h"

std::vector<std::string> get_representative_uids(std::string path){
    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    nameGenerator->SetDirectory(path);

    return nameGenerator->GetSeriesUIDs();
}

std::optional<ImageType::Pointer> get_representative_series_image(std::string path,std::string seriesIdentifier)
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

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    if(fileNames.size()==0)
        return std::nullopt;
    
    // we choose to select the filename in the middle

    size_t index = (fileNames.size()==1) ? 1 : std::round(fileNames.size()/2.0);

    FileNamesContainer single_image_container;
    single_image_container.push_back(fileNames[index]);
    reader->SetFileNames(single_image_container);
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


    auto query = [&](const std::string& entryID){
        auto tagItr = dictionary.Find(entryID);
        std::optional<std::string> tagvalue = std::nullopt;
        if (tagItr == dictionary.End()){  
            std::cout << "Tag " << entryID;
            std::cout << " not found in the DICOM header" << std::endl;
            return tagvalue;
        }
        MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType *>(tagItr->second.GetPointer());

        if (entryvalue)
            tagvalue = entryvalue->GetMetaDataObjectValue();
        //    std::cout << "Patient's (" << entryID << ") ";
        //    std::cout << " is: " << *tagvalue << std::endl;
        //}
        //else
        //    std::cout << "Entry was not of string type" << std::endl;
        return tagvalue;
    };

    auto bits_stored_atribute = query("0028|0101");  //Bits Stored Attribute

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<DICOMImageType, DICOMImageType>;
    
    auto compute_conversion = [](const std::string& number_in_string){
        std::optional<int> result = std::nullopt;
        int result_l{};
        auto [ptr, ec] = std::from_chars(number_in_string.data(), number_in_string.data() + number_in_string.size(), result_l);
 
        if (ec == std::errc())
            result = result_l;
        
        return result;
    };


    int bits_stored = bits_stored_atribute ? (compute_conversion(*bits_stored_atribute) ?  *compute_conversion(*bits_stored_atribute) : 16) : 16;  //Bits Allocated Attribute

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<DICOMImageType, DICOMImageType>;
    auto shiftFilter = ShiftScaleFilterType::New();
    
    shiftFilter->SetScale(std::pow(2,sizeof(PixelType)*8.0-bits_stored));
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

std::optional<ImageType::Pointer> load_volume_from_selected_uid(std::string path,std::string seriesIdentifier){
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

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    if(fileNames.size()==0)
        return std::nullopt;
    
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


    auto query = [&](const std::string& entryID){
        auto tagItr = dictionary.Find(entryID);
        std::optional<std::string> tagvalue = std::nullopt;
        if (tagItr == dictionary.End()){  
            std::cout << "Tag " << entryID;
            std::cout << " not found in the DICOM header" << std::endl;
            return tagvalue;
        }
        MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType *>(tagItr->second.GetPointer());

        if (entryvalue)
            tagvalue = entryvalue->GetMetaDataObjectValue();
        //    std::cout << "Patient's (" << entryID << ") ";
        //    std::cout << " is: " << *tagvalue << std::endl;
        //}
        //else
        //    std::cout << "Entry was not of string type" << std::endl;
        return tagvalue;
    };

    auto patient_orientation = query("0020|0020");  //Patient Orientation Attribute
    auto image_laterality = query("0020|0062");  //Image Laterality Attribute
    auto slice_thickness = query("0018|0050");  //Slice Thickness Attribute
    auto spacing_between_slices = query("0018|0088");  //Spacing Between Slices Attribute
    auto image_position = query("0020|0032");  //Image Position (Patient) Attribute
    auto image_orientation = query("0020|0037");  //Image Orientation (Patient) Attribute
    auto anatomical_orientation_type = query("0010|2210");  //Anatomical Orientation Type
    

    auto print_val = [](std::string description,std::optional<std::string> contained){
        if(contained)
            std::cout << "the requested atribute (" << description << ") has the value: " << *contained << "\n";
        else 
            std::cout << "the requested atribute (" << description << ") has no value\n";
    };

    print_val("patient_orientation",patient_orientation);
    print_val("image_laterality",image_laterality);
    print_val("slice_thickness",slice_thickness);
    print_val("spacing_between_slices",spacing_between_slices);
    print_val("image_position",image_position);
    print_val("image_orientation",image_orientation);
    print_val("anatomical_orientation_type",anatomical_orientation_type);

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<DICOMImageType, DICOMImageType>;
    auto shiftFilter = ShiftScaleFilterType::New();

    auto bits_stored_atribute = query("0028|0101");  //Bits Stored Attribute

    using ShiftScaleFilterType = itk::ShiftScaleImageFilter<DICOMImageType, DICOMImageType>;
    
    auto compute_conversion = [](const std::string& number_in_string){
        std::optional<int> result = std::nullopt;
        int result_l{};
        auto [ptr, ec] = std::from_chars(number_in_string.data(), number_in_string.data() + number_in_string.size(), result_l);
 
        if (ec == std::errc())
            result = result_l;
        
        return result;
    };


    int bits_stored = bits_stored_atribute ? (compute_conversion(*bits_stored_atribute) ?  *compute_conversion(*bits_stored_atribute) : 16) : 16;  //Bits Allocated Attribute


    shiftFilter->SetScale(std::pow(2,sizeof(PixelType)*8.0-bits_stored));
    shiftFilter->SetShift(0);
    shiftFilter->SetInput(reader->GetOutput());

    using FilterType = itk::CastImageFilter<DICOMImageType, ImageType>;
    auto filter = FilterType::New();
    filter->SetInput(shiftFilter->GetOutput());

    itk::OrientImageFilter<ImageType,ImageType>::Pointer orienter =itk::OrientImageFilter<ImageType,ImageType>::New();
    orienter->UseImageDirectionOn();
    orienter->SetDesiredCoordinateOrientation(itk::SpatialOrientation::ITK_COORDINATE_ORIENTATION_RAI);
     
    orienter->SetInput(filter->GetOutput());
    
    try
    {
        orienter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return std::nullopt;
    }
    return orienter->GetOutput();
}