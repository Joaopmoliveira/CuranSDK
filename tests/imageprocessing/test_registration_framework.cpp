#include <iostream>
#include <string>

#include "imageprocessing/RegistrationUS_CT.h"
#include "itkImageFileReader.h"

using ImageReaderType = itk::ImageFileReader<ImageType>;

int main(int argc, char **argv){
    std::string path_fixed{CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha"};
    std::string path_moving{"C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha"};

    std::printf("\nReading input volumes...\n");
    auto fixedImageReader = ImageReaderType::New();
    auto movingImageReader = ImageReaderType::New();

    fixedImageReader->SetFileName(path_fixed);
    movingImageReader->SetFileName(path_moving);
    try
    {
        fixedImageReader->Update();
        movingImageReader->Update();
    }
    catch(const itk::ExceptionObject &err){
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err.GetDescription() << std::endl;
        return 1;
    }

    ImageType::Pointer pointer2inputfixedimage = fixedImageReader->GetOutput();
    ImageType::Pointer pointer2inputmovingimage = movingImageReader->GetOutput();
    register_volumes(pointer2inputfixedimage,pointer2inputmovingimage);
    return 0;
}