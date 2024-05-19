#include <cstring>
#include <cstdlib>
#include <string>
#include <iostream>

#include <cassert>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <memory>

struct shm_remove
{
	shm_remove() { boost::interprocess::shared_memory_object::remove("KAZAMAS"); }
	~shm_remove(){ boost::interprocess::shared_memory_object::remove("KAZAMAS"); }
};


struct SharedMemoryCreator{
private:
	shm_remove remover;
	boost::interprocess::shared_memory_object shm;
	boost::interprocess::mapped_region region;
	explicit SharedMemoryCreator() : remover{},shm{boost::interprocess::create_only, "KAZAMAS", boost::interprocess::read_write}{
		shm.truncate(100);
		region = boost::interprocess::mapped_region{shm, boost::interprocess::read_write};
}
public:
	static std::unique_ptr<SharedMemoryCreator> create(){
		std::unique_ptr<SharedMemoryCreator> unique = std::unique_ptr<SharedMemoryCreator>(new SharedMemoryCreator{});
		return unique;
}

	~SharedMemoryCreator(){
	}

	inline unsigned char* get_shared_memory_address(){
		return static_cast<unsigned char*>(region.get_address());
	}
};

struct SharedMemoryAccessor{
private:
	boost::interprocess::shared_memory_object shm;
	boost::interprocess::mapped_region region;

	explicit SharedMemoryAccessor() :shm{boost::interprocess::open_only, "KAZAMAS", boost::interprocess::read_only},region{shm, boost::interprocess::read_only}{
	}

public:

	static std::unique_ptr<SharedMemoryAccessor> create(){
		std::unique_ptr<SharedMemoryAccessor> unique = std::unique_ptr<SharedMemoryAccessor>(new SharedMemoryAccessor{});
		return unique;
	}

	~SharedMemoryAccessor(){

	}

	unsigned char* get_shared_memory_address(){
		return static_cast<unsigned char*>(region.get_address());
	}
};


int main(int argc, char *argv[])
{
   using namespace boost::interprocess;

   if(argc == 1){  //Parent process
      
      auto shared_mem = SharedMemoryCreator::create();
      auto pointer = shared_mem->get_shared_memory_address();
      //Write all the memory to 1
      std::memset(pointer, 1, 100);

      //Launch child process
      std::string s(argv[0]); s += " child ";
      if(0 != std::system(s.c_str())){
        std::cout << "failure\n";
        return 1;
      }

   }
   else{
      auto shared_mem = SharedMemoryAccessor::create();
      auto pointer = shared_mem->get_shared_memory_address();
      //Check that memory was initialized to 1
      for(std::size_t i = 0; i < 100; ++i)
         if(*pointer++ != 1)
            return 1;   //Error checking memory
   }
   return 0;
}