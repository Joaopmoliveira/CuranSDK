#include <iostream>
#include <optional>
#include <string>

std::optional<FixedImageType::Pointer> read_volume_from_directory(const std::string &path)
{
    using ReaderType = itk::ImageSeriesReader<FixedImageType>;
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

    fileNames = nameGenerator->GetFileNames(nameGenerator->GetSeriesUIDs().front());

    if (fileNames.size() == 0)
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
    return reader->GetOutput();
}

int main(int argc, char* argv[]){
    if(argc!=5){
        std::cout << "To run the executable you must provide four arguments";
    }

    std::string input_volume{argv[1]};
    std::string output_volume{argv[2]};
    std::string close_to_origin{argv[3]};
    std::string away_from_origin{argv[4]};



    return 0;
}