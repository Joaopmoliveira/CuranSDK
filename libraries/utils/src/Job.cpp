#include "utils/Job.h"

namespace curan{
namespace utilities{

Job::Job(std::string descript, std::function<void(void)> funct) : description{ descript }, function_to_execute{ funct }
{}

Job::Job()
{}

}
}