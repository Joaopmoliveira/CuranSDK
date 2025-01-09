#include "utils/Logger.h"
#include "utils/TheadPool.h"

int main()
{
try{
    
    auto pool = curan::utilities::ThreadPool::create(1,curan::utilities::TERMINATE_ALL_PENDING_TASKS);
    curan::utilities::Logger logger{};
    pool->submit("printing function",[&](){
        while(logger)
            logger.processing_function();
    });
    curan::utilities::print<curan::utilities::info>("Hello {}!\n", "world");
    int a = 12;
    curan::utilities::print<curan::utilities::info>("the address is: {:d} value {:d}\n",(uintptr_t)&a,a);
    std::cin >> a;
    curan::utilities::print<curan::utilities::info>("the address is: {:d} value {:d}\n",(uintptr_t)&a,a);
    return 0;
} catch(std::runtime_error& e){
    return 1;
} catch(...){
    return 2;
} 
}