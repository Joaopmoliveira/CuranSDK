#ifndef CURAN_MODIFY_XML_FIELD_HEADER_FILE_
#define CURAN_MODIFY_XML_FIELD_HEADER_FILE_

#include <filesystem>
#include <string>

namespace curan {
namespace utilities {

/*
Error code returned when modifying the xml file desired by the user
*/
enum ErrorCode{
    SUCCESS = 0,
    PATH_NOT_FOUND,
    CHILD_NOT_FOUND,
    ATRIBUTE_NOT_FOUND,
    INVALID_ARGUMENT,
};

/*
The function takes the given identifier and path, parses the XML file to make sure everything is proper
then it goes along the nested children specified in the field nesting, finds the desired atribute in the 
last level of that nesting, and takes the identifier tag, compares it to the desired identifier and replaces the old value with 
the new value plus the old value
*/
[[nodiscard]] ErrorCode modify_xml_field_in_place(const std::filesystem::path& path,std::string atribute_name ,std::string identifier_tag,std::string desired_identifier,double new_value, const std::vector<std::string>& nesting);

}
}

#endif