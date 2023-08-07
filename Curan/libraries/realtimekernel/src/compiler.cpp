#include <map>
#include <string>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <memory>
#include <mutex>
#include <fstream>
#include <iostream>
#include <vector>

enum types{
    DOUBLE ,
    FLOAT,
    INT,
    SIZE_T,
    BYTES
};

std::map<std::string,types> known_types= {
    {"double",types::DOUBLE},
    {"float",types::FLOAT},
    {"int",types::INT},
    {"size_t",types::SIZE_T},
    {"bytes",types::BYTES}
};

char header_begin[] = R"(
#include <cassert>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <memory>
)";

struct field_description{
    std::string name;
    std::string type_name;
    size_t array = 0;
    size_t type_size = 0;
    types internal_type;
    size_t adress;
};

int main(int argc, char* argv[]){
    std::cout << "the compiler generates two header files,\n the header file which creates the shared memory and the header file which \n simply accesses the shared memory." << std::endl;
    std::stringstream header_file;
    header_file << header_begin;
    if(argc!=2){
        std::cout << "need a .json file to process, please supply it as an argument" << std::endl;
        return 1;
    }

    std::filesystem::path config_file;
    try{
        config_file = std::filesystem::path{argv[1]};
    } catch (...){
        std::cout << "failed to open the supplied file" << std::endl;
        return 1;
    }
    nlohmann::json configuration_data;
    try{
        configuration_data = nlohmann::json::parse(std::ifstream(config_file));
    } catch (...){
        std::cout << "failed to parse the json file" << std::endl;
        return 1;
    }

    nlohmann::json messages;
    size_t global_memory_index = 0;
    try{
        messages = configuration_data["messages"];
    } catch (...){
        std::cout << "failed to find any messages field in the supplied json file" << std::endl;
        return 1;
    }
    //lets find the message name
    for(const auto & message : messages){
        std::stringstream local_class_stream;

        std::string class_name;
        try{
            class_name = message["message"];
        } catch (...){
            std::cout << "the name of a supplied message is not present" << std::endl;
            return 1;
        }
        // we need two classes for each type, a layout and the actual container
        // and we need two functions, a serializer and a deserializer
        std::vector<field_description> fiels;
        nlohmann::json contained_fields;
        try{
            contained_fields = message["fields"];
        } catch (...){
            std::cout << "the message " << class_name << " does not contain any fields" << std::endl;
            return 1;
        }

        for(const auto & field : contained_fields){
            field_description description;
            std::string type;
            try{
                description.name = field["name"];
                type = field["type"];
                description.array = field["array"];
            } catch (...){
                std::cout << "all fiels must contain a name, a type and a number specifiying the multiplicity of the field" << std::endl;
                return 1;
            }

            if (auto search = known_types.find(type); search != known_types.end()){
                description.internal_type = search->second;
                
            } else {
                std::cout << "found type which I don't understand, the field (" << description.name << ") contains the unknown type (" << type << "). stoping compilation" << std::endl;
                return 1;
            }
            switch(description.internal_type){
                case types::BYTES:
                description.type_size = 1;
                description.type_name = "unsigned char";
                break;
                case types::DOUBLE:
                description.type_size = 8;
                description.type_name = "double";
                break;
                case types::FLOAT:
                description.type_size = 4;
                description.type_name = "float";
                break;
                case types::INT:
                description.type_size = 4;
                description.type_name = "int";
                break;
                case types::SIZE_T:
                description.type_size = 8;
                description.type_name = "size_t";
                break;
                default:
                throw std::runtime_error("supplied type is unknown");
                break;
            }
            fiels.push_back(description);
        }

        //first we print the class itself on the string stream
        local_class_stream << "struct " << class_name << "\n{";
        for(const auto& field : fiels){
            if(field.array==1){
                local_class_stream << "\t" << field.type_name << " " << field.name << ";\n";
                continue;
            }
            if(field.internal_type!=types::BYTES){
                local_class_stream << "\t" << field.type_name << " " << field.name << "[" << field.array << "];\n";
                continue;
            }
            local_class_stream << "\t" << field.type_name << "* " << field.name << " = nullptr;\n";
        }
                            
        local_class_stream << "};\n\n";

        // now layout of our class thus this layout will be called class_name_layout
        local_class_stream << "struct " << class_name << "_layout \n{";
        for(auto& field : fiels){
            local_class_stream << "\t size_t " << field.name << "_address = " << global_memory_index << ";\n";
            field.adress = global_memory_index;
            global_memory_index += field.type_size*field.array;
            local_class_stream << "\t size_t " << field.name <<  "_size = " << field.type_size*field.array << ";\n\n";
        }
        local_class_stream << "};\n\n";

        // now we print the function which maps our reading into our blob of memory
        local_class_stream << "void copy_from_" << class_name << "_to_shared_memory( unsigned char* memory , const " << class_name <<  " & tmp)\n"
                            << "{ \n\tconstexpr " << class_name << "_layout mapping;\n\n";
        for(auto & field : fiels){
           if(field.internal_type == types::BYTES){ //we need to check if the memory is distict from nullptr in the case of byte array
            local_class_stream << "\tassert( tmp." << field.name << "!=nullptr);\n"; 
            local_class_stream << "\tstd::memcpy( memory+mapping." << field.name << "_address , tmp." << field.name<< " , mapping."<< field.name << "_size );\n\n";
            continue;
           }
           local_class_stream << "\tstd::memcpy( memory+mapping." << field.name << "_address , &tmp." << field.name<< " , mapping."<< field.name << "_size );\n\n";
        };
                           
        local_class_stream << "}" << std::endl;

        // now we print the function which maps our shared memory into our reading
        local_class_stream << "\n\nvoid copy_from_shared_memory_to_" << class_name << "( const unsigned char*  memory" <<  "," << class_name << " & tmp)\n" 
                           << "{ \n\tconstexpr " << class_name << "_layout mapping;\n\n";
        for(auto & field : fiels){
           if(field.internal_type == types::BYTES){ //we need to check if the memory is distict from nullptr in the case of byte array
            local_class_stream << "\tassert( tmp." << field.name << "!=nullptr);\n"; 
            local_class_stream << "\tstd::memcpy( tmp." << field.name<< ",memory+mapping." << field.name << "_address , mapping."<< field.name << "_size );\n\n";
            continue;
           }
           local_class_stream << "\tstd::memcpy( &tmp." << field.name<< ",memory+mapping." << field.name << "_address , mapping."<< field.name << "_size );\n\n";
        };
        local_class_stream << "}" << std::endl;
        header_file << local_class_stream.str();
    }

    std::string shared_memory_name;
    try{
        shared_memory_name = configuration_data["shared_memory_name"];
    } catch (...){
        std::cout << "you must supply the name of the block of shared memory you want to use" << std::endl;
        return 1;
    }

    std::stringstream out_header_file_access;
    out_header_file_access <<  header_file.str() 
                           << "struct SharedMemoryAccessor{\n"
                           << "private:\n"
                           << "\tboost::interprocess::shared_memory_object shm;\n"
                           << "\tboost::interprocess::mapped_region region;\n\n"
                           << "\texplicit SharedMemoryAccessor() :shm{boost::interprocess::open_only, \"" << shared_memory_name << "\", boost::interprocess::read_write}{\n"
                           << "\t\tregion = boost::interprocess::mapped_region{shm, boost::interprocess::read_write};\n"
                           << "\t}\n\n"
                           << "public:\n\n"
                           << "\tstatic std::unique_ptr<SharedMemoryAccessor> create(){\n"
                           << "\t\tstd::unique_ptr<SharedMemoryAccessor> unique = std::unique_ptr<SharedMemoryAccessor>(new SharedMemoryAccessor{});\n"
                           << "\t\treturn unique;\n"
                           << "\t}\n\n"
                           << "\t~SharedMemoryAccessor(){\n\n"
                           << "\t}\n\n"
                           << "\tunsigned char* get_shared_memory_address(){\n"
                           << "\t\treturn static_cast<unsigned char*>(region.get_address());\n"
                           << "\t}\n"
                           << "};"  << std::endl;

    std::stringstream out_header_file_create;
    out_header_file_create <<  header_file.str()
                           << "struct shm_remove\n"
                           << "{\n"
                           << "\tshm_remove() { boost::interprocess::shared_memory_object::remove(\"" << shared_memory_name << "\"); }\n"
                           << "\t~shm_remove(){ boost::interprocess::shared_memory_object::remove(\"" << shared_memory_name << "\"); }\n"
                           << "};\n"
                           << "\n\nstruct SharedMemoryCreator{\n"
                           << "private:\n"
                           << "\tshm_remove remover;\n"
                           << "\tboost::interprocess::shared_memory_object shm;\n"
                           << "\tboost::interprocess::mapped_region region;\n"
                           << "\texplicit SharedMemoryCreator() : remover{},shm{boost::interprocess::create_only, \"" << shared_memory_name << "\", boost::interprocess::read_write}{\n"
                           << "\t\tshm.truncate("<<global_memory_index<<");\n"
                           << "\t\tregion = boost::interprocess::mapped_region{shm, boost::interprocess::read_write};\n"
                           << "}\n"
                           << "public:\n"
                           << "\tstatic std::unique_ptr<SharedMemoryCreator> create(){\n"
                           << "\t\tstd::unique_ptr<SharedMemoryCreator> unique = std::unique_ptr<SharedMemoryCreator>(new SharedMemoryCreator{});\n"
                           << "\t\treturn unique;\n"
                           << "}\n\n"
                           << "\t~SharedMemoryCreator(){\n"
                           << "\t}\n\n"

                           << "\tinline unsigned char* get_shared_memory_address(){\n"
                           << "\t\treturn static_cast<unsigned char*>(region.get_address());\n"
                           << "\t}\n"
                           << "};" << std::endl;

    std::ofstream ostrmacess("header_acessor.h", std::ios::out);
    ostrmacess << out_header_file_access.str() << std::endl;

    std::ofstream ostrmcreate("header_creator.h", std::ios::out);
    ostrmcreate << out_header_file_create.str() << std::endl;

    return 0;
}