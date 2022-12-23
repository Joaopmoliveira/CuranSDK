#ifndef CURAN_CONNECTION_HEADER_FILE_
#define CURAN_CONNECTION_HEADER_FILE_

#include <mutex>
#include <memory>
#include <list>
#include <functional>

namespace curan {
	namespace utils {

        template<typename signal>
        class Callable;

        template<typename signal>
        struct CallableConnectInfo {
            std::shared_ptr<Callable<signal>> source1;
            std::function<int(signal)> source1_called;
            std::shared_ptr<Callable<signal>> source2;
            std::function<int(signal)> source2_called;
            /*
            * Take into considerations for the lamda calleds1 and calleds2, not to store a copy
            * of the shared ptr s1 and s2 because this will create a ciclic dependency which will
            keep in memory the objects s1 and s2.
            */
            CallableConnectInfo(std::shared_ptr<Callable<signal>> s1, std::function<int(signal)>calleds1, std::shared_ptr<Callable<signal>> s2, std::function<int(signal)>calleds2) {
                source1 = s1;
                source2 = s2;
                source1_called = calleds1;
                source2_called = calleds2;
            }
        };

        /*
        When calling 
        */
        template<typename signal>
        void connect_callables(CallableConnectInfo<signal> info);


        /*
        The callable template class allows one to connect different objects and
        inject functionality to their interaction. When you wish to use, just derive
        your class from the callable class and provide the type of signal which will be
        passed between the objetcs. When the callable object is killed, to avoid after
        use problems, we destroy the common weak pointer references between two callable objects.
        After the destructor has been called no further communication can be established.
        */
        template<typename signal>
        class Callable {
            /*
            List of objects which are currently connected to this object.
            When the object throws a signal it warns all the callable objects
            in this list.
            */
            std::list<std::weak_ptr<Callable<signal>>> warnable;
            /*
            If we wish the add functionality
            */
            std::function<int(signal)> called_function;
            std::mutex mut;

            friend void connect_callables<signal>(CallableConnectInfo<signal> info);

        public:
            Callable() {
            }

            /*
            The call function goes through the current objects which it thinks
            are connected to the callable object. If if finds one which has been destroyed in the
            meantime it will erase it from the list of objects which it thinks are connected
            to it.
            */
            void call(signal sig) {
                std::lock_guard<std::mutex> g(mut);
                warnable.remove_if([](std::weak_ptr<Callable<signal>> sig) { return sig.expired(); });
                for (auto iter = warnable.begin(); iter != warnable.end(); ++iter) {
                    if (std::shared_ptr<Callable<signal>> spt = iter->lock()) {
                        std::lock_guard<std::mutex> g1(spt->mut);
                        spt->called_function(sig);
                    }
                }
            }

            /*
            Replace the existing functional call for the new supplied function
            */
            void replace(std::function<int(signal)> functional = nullptr) {
                std::lock_guard<std::mutex> g(mut);
                called_function = functional;
            }
        };

        template<typename signal>
        void connect_callables(CallableConnectInfo<signal> info) {
            std::weak_ptr<Callable<signal>> s1 = info.source1;
            std::weak_ptr<Callable<signal>> s2 = info.source2;
            std::lock_guard<std::mutex> g1(info.source1->mut);
            std::lock_guard<std::mutex> g2(info.source2->mut);
            info.source1->called_function = info.source1_called;
            info.source1->warnable.push_back(s2);
            info.source2->called_function = info.source2_called;
            info.source2->warnable.push_back(s1);
        };

		
	}
}

#endif