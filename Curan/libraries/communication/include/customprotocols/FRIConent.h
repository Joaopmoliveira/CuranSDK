

namespace curan{
namespace communication{

    struct FRIContents{
        size_t message_size;
        std::vector<double> angles;
        std::vector<double> external_torques;
        std::vector<double> measured_torques;
        size_t number_of_links;

        inline size_t get_header_size(){
            return sizeof(size_t);
        }

        inline size_t get_body_size(){
            return message_size;
        }
    };

}
}