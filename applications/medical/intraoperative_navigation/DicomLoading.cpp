#include "DicomLoading.h"

InputImageType::Pointer load_dicom(const std::string &path)
{
    using ImageReaderType = itk::ImageFileReader<InputImageType>;

    std::printf("\nReading input volumes...\n");
    auto fixedImageReader = ImageReaderType::New();

    fixedImageReader->SetFileName(path);

    try
    {
        fixedImageReader->Update();
        return fixedImageReader->GetOutput();
    }
    catch (const itk::ExceptionObject &err)
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
    }
    return nullptr;
};