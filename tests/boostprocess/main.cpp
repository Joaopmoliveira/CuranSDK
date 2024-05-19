#include <iostream>
#include <string>
#include <boost/process.hpp>
#include <memory>

int main()
{
try{
    
    std::string cmd = "notepad.exe";
    boost::process::ipstream out;
    std::unique_ptr<boost::process::child> c = std::make_unique<boost::process::child>(cmd, boost::process::std_out > out);
    c->wait();
    std::cout << "exit code: " << c->exit_code() << std::endl; 
    return 0;
} catch(...){
    return 1;
}
}