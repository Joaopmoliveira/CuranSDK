#include "utils/Job.h"

namespace curan{
namespace utilities{

Job::Job(std::string descript, std::function<void(void)> funct) : function_to_execute{ funct }, description{ descript }
{}

Job::Job()
{}

}
}