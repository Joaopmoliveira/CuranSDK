#include "utils/Job.h"

namespace curan{
namespace utilities{

Job::Job(std::string descript, std::function<void(void)> funct) : m_function_to_execute{ funct }, m_description{ descript }
{}

Job::Job()
{}

}
}