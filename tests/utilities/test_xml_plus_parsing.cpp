#include "utils/ModifyXMLField.h"
#include <iostream>

int main(){
    const std::string device_id{"ROBOT"};
    switch(curan::utilities::modify_xml_field_in_place(CURAN_COPIED_RESOURCE_PATH"/plus_config/plus_spacial_calib_robot_xml/robot_image.xml","LocalTimeOffsetSec","Id",device_id,10,{"PlusConfiguration","DataCollection"})){
        case curan::utilities::ErrorCode::SUCCESS:
            std::cout << "success" << std::endl;
        break;
        case curan::utilities::ErrorCode::CHILD_NOT_FOUND:
            std::cout << "data collection not found" << std::endl;
        break;
        case curan::utilities::ErrorCode::INVALID_ARGUMENT:
            std::cout << "invalid argument" << std::endl; 
        break;
        case curan::utilities::ErrorCode::PATH_NOT_FOUND:
            std::cout << "path not found" << std::endl;
        break;
        case curan::utilities::ErrorCode::ATRIBUTE_NOT_FOUND:
            std::cout << "device id: " << device_id << " - device not found" << std::endl;
        break;
        default:
            std::cout << "unknown error" << std::endl;
    }
    return 0;
}