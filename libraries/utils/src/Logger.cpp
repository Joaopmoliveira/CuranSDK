#include "utils/Logger.h"

namespace curan{
namespace utilities{

    namespace detail{
        Logger *cout = nullptr;
    }

    Logger::Logger() : running{true}, terminated{true}{
        detail::cout = this;
    };

    void Logger::processing_function()
    {
        std::deque<std::string> local_data_queue;
        std::unique_lock<std::mutex> lk(m_mut);
        if(running){
            terminated = false;
            if(m_data_queue.size()==0 && running)
                cv.wait(lk, [this] { return m_data_queue.size()!=0 || !running; });
            local_data_queue = std::move(m_data_queue);
            m_data_queue = std::deque<std::string>{};
        }
        terminated = true;
        lk.unlock();
        cv.notify_one();
        for (const auto &to_print : local_data_queue)
            std::cout << to_print;

    }

    Logger::~Logger()
    {
        std::unique_lock<std::mutex> lk(m_mut);
        running = false;
        detail::cout = nullptr; 
        lk.unlock();
        cv.notify_one();
        lk.lock();
        cv.wait(lk, [this] { return terminated; });
    };

}
}