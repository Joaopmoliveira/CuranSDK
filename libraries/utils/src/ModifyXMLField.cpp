#include "utils/ModifyXMLField.h"
#include "pugixml.hpp"
#include <iomanip>
#include <charconv>
#include <sstream>
#include <fstream>

namespace curan {
namespace utilities {

ErrorCode modify_xml_field_in_place(const std::filesystem::path& path,std::string atribute_name ,std::string identifier_tag,std::string desired_identifier,double new_value, const std::vector<std::string>& nesting){
    bool found_the_desired_device = false;
    std::stringstream modified_file;
    {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(path.c_str());
        if(!result)
            return PATH_NOT_FOUND;
        pugi::xml_node desired_node = doc;
        for(const auto& val : nesting){
            desired_node = desired_node.child(val.c_str());
            if(!desired_node)
                return CHILD_NOT_FOUND;
        }
        for(auto val : desired_node.children()){
            for(auto atrib : val.attributes()){
                if(!std::string{atrib.name()}.compare(atribute_name) && !std::string{val.attribute(identifier_tag.c_str()).value()}.compare(desired_identifier)){
                    std::string value_before{atrib.value()};
                    double contained_value = 0.0;
                    auto [ptr, ec] = std::from_chars(value_before.data(), value_before.data() + value_before.size(), contained_value);
 
                    if (ec == std::errc())
                        found_the_desired_device = true;
                    else if (ec == std::errc::invalid_argument)
                        return INVALID_ARGUMENT;
                    else if (ec == std::errc::result_out_of_range)
                        return INVALID_ARGUMENT;
                    if(found_the_desired_device){
                        auto strin = std::to_string(contained_value+new_value);
                        atrib.set_value(strin.data());
                        break;
                    }

                }
            }
            if(found_the_desired_device)
                break;
        }
        doc.print(modified_file);
    }
    
    if (!found_the_desired_device) 
        return ATRIBUTE_NOT_FOUND;


    std::ofstream output{path};
    if(!output.is_open()){
        return PATH_NOT_FOUND;
    }
    output << modified_file.str();
    
    return SUCCESS;
}

}
}