#include <iostream>
#include <string>
#include <chrono>
#include <thread>

class MemoryOwner // this is a naive implementation of a memory owner, simillar to a shared pointer
{
  struct ControlBlock
  {
    std::atomic<size_t> counter = 1;

    void increment()
    {
      counter.fetch_add(1);
    }

    bool decrement()
    {
      return counter.fetch_sub(1)-1 == 0;
    }
  };
  public:
  void *pointer_to_data;
  size_t block_of_memory;

  MemoryOwner() : pointer_to_data{nullptr}, control_block{nullptr}
  {
    std::cout << "alocated MemoryOwner()" << std::endl;
    control_block = new ControlBlock{};
    print_addresses();
  }

  MemoryOwner(size_t block_size) : pointer_to_data{nullptr}, control_block{nullptr}
  {
    std::cout << "alocated MemoryOwner(size_t block_size)" << std::endl;
    control_block = new ControlBlock{};
    pointer_to_data = new unsigned char[block_size]{};
    block_of_memory = block_size;
    print_addresses();
  }

  MemoryOwner(const MemoryOwner &other) : pointer_to_data{nullptr}, control_block{nullptr}
  {
    std::cout << "alocated MemoryOwner(const MemoryOwner &other)" << std::endl;
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    print_addresses();
  }

  MemoryOwner &operator=(const MemoryOwner &other)
  {
    std::cout << "copied MemoryOwner &operator=(const MemoryOwner &other)" << std::endl;
    decrement();
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    print_addresses();
  }

  MemoryOwner(MemoryOwner &&other) : pointer_to_data{nullptr}, control_block{nullptr}
  {
    std::cout << "alocated MemoryOwner(MemoryOwner &&other)" << std::endl;
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    print_addresses();
  }

  MemoryOwner &operator=(MemoryOwner &&other)
  {
    std::cout << "copied MemoryOwner &operator=(MemoryOwner &&other)" << std::endl;
    decrement();
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    print_addresses();
  }

  ~MemoryOwner()
  {
    decrement();
    std::cout << "destructor ~MemoryOwner()" << std::endl;
  }

  void Alloc(size_t size)
  {
    decrement();
    control_block = new ControlBlock{};
    pointer_to_data = new unsigned char[size]{};
    block_of_memory = size;
    std::cout << "raw Alloc(size_t size)" << std::endl;
    print_addresses();
  }

  void* data(){
    return pointer_to_data;
  }

  size_t size(){
    return block_of_memory;
  }

private:
  void decrement()
  {
    if (control_block->decrement())
    {
      print_addresses();
      if (pointer_to_data){
        std::cout << "decrement(): deleting block of memory" << std::endl;
        delete[] pointer_to_data;
      }
      std::cout << "decrement(): deleting control block" << std::endl;
      delete control_block;
    }
    pointer_to_data = nullptr;
    control_block = nullptr;
  }
  ControlBlock *control_block;

  void print_addresses(){
    std::printf("(this: %p) (pointer_to_data:%p) (control_block:%p)\n",this,pointer_to_data,control_block);
  }
};

int main(){
    try{
        MemoryOwner owner;
        MemoryOwner owner2{10};
        MemoryOwner copied_owner{owner};
        MemoryOwner copied_owner2{owner2};
    } catch (std::runtime_error &e){
        std::cout << "exception: " << e.what() << std::endl;
        return 2;
    } catch (...) {
        std::cout << "generic failure" << std::endl;
        return 1;
    }
    return 0;
}