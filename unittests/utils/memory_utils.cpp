#include <gtest/gtest.h>
#include "utils/MemoryUtils.h"

auto comparator = [](const std::string &value_to_control, const std::shared_ptr<curan::utilities::MemoryBuffer> &buffer)
{
  size_t address = 0;
  for (auto begin = buffer->begin(); begin != buffer->end(); ++begin)
  {
    size_t local_address = 0;
    for (size_t internal_address = 0; internal_address < begin->size(); ++internal_address, ++address)
      if (value_to_control.at(address) != *(((char *)begin->data()) + internal_address))
        return false;
  }
  return true;
};

const std::string ref_value_to_control = "1_2_3_4_5_6_7_8_9_10_11_12_13_14_15_16_17_18_19_20_21_22_23_24_25_26_27_28_29_30_31_32_33_34_35_36_";

TEST(UnitTestMemoryUtilities, CopyBufferLogic)
{
  std::string value_to_control = "1_2_3_4_5_6_7_8_9_10_11_12_13_14_15_16_17_18_19_20_21_22_23_24_25_26_27_28_29_30_31_32_33_34_35_36_";
  // the copy buffer receives an pointer and the size of data to be copied and does so promply
  auto buff_of_interest = curan::utilities::CopyBuffer::make_shared(value_to_control.data(), value_to_control.size());
  ASSERT_EQ(comparator(ref_value_to_control, buff_of_interest), true) << "the buffers must be the same";
  value_to_control[0] = 'a';
  ASSERT_EQ(comparator(ref_value_to_control, buff_of_interest), true) << "the buffers must be the same even when we change the memory of the original block of memory";
}

/*
this is a naive implementation of a memory owner, 
simillar to a shared pointer. Basically there can
be multiple copies of this object and when the last
one dies, the memory that it owns is killed
*/
class MemoryOwner 
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
  unsigned char *pointer_to_data;
  size_t block_of_memory;

  MemoryOwner() : pointer_to_data{nullptr}, control_block{nullptr}, block_of_memory{0}
  {
    control_block = new ControlBlock{};
  }

  MemoryOwner(size_t block_size) : pointer_to_data{nullptr}, control_block{nullptr}, block_of_memory{0}
  {
    control_block = new ControlBlock{};
    pointer_to_data = new unsigned char[block_size]{};
    block_of_memory = block_size;
  }

  MemoryOwner(const MemoryOwner &other) : pointer_to_data{nullptr}, control_block{nullptr}, block_of_memory{0}
  {
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    block_of_memory = other.block_of_memory;
  }

  MemoryOwner &operator=(const MemoryOwner &other)
  {
    decrement();
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    block_of_memory = other.block_of_memory;
  }

  MemoryOwner(MemoryOwner &&other) : pointer_to_data{nullptr}, control_block{nullptr}, block_of_memory{0}
  {
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    block_of_memory = other.block_of_memory;
  }

  MemoryOwner &operator=(MemoryOwner &&other)
  {
    decrement();
    control_block = other.control_block;
    control_block->increment();
    pointer_to_data = other.pointer_to_data;
    block_of_memory = other.block_of_memory;
  }

  ~MemoryOwner()
  {
    decrement();
  }

  unsigned char& operator [](size_t index) {
    if(index>=block_of_memory)
      std::terminate();
    return pointer_to_data[index];
  }

  void Alloc(size_t size)
  {
    decrement();
    control_block = new ControlBlock{};
    pointer_to_data = new unsigned char[size]{};
    block_of_memory = size;
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
      if (pointer_to_data){
        delete[] pointer_to_data;
      }
      delete control_block;
    }
    pointer_to_data = nullptr;
    control_block = nullptr;
  }
  ControlBlock *control_block;
};

TEST(UnitTestMemoryUtilities, CaptureBufferLogic)
{
  MemoryOwner owner{ref_value_to_control.size()};
  for(size_t i = 0; i < ref_value_to_control.size(); ++i)
    owner[i] = ref_value_to_control[i];
  auto buff_of_interest = curan::utilities::CaptureBuffer::make_shared(owner.data(), owner.size(), owner);

  ASSERT_EQ(comparator(ref_value_to_control, buff_of_interest), true) << "the buffers must be the same";
  owner[0] = 'a';
  ASSERT_NE(comparator(ref_value_to_control, buff_of_interest), true) << "the buffers must be the different because we messed with the memory of the string that is the owner of the block of memory";
}

TEST(UnitTestMemoryUtilities, CaptureBufferLogicAfterLife)
{
  std::shared_ptr<curan::utilities::MemoryBuffer> buff_of_interest;
  {
    MemoryOwner owner{ref_value_to_control.size()};
    for(size_t i = 0; i < ref_value_to_control.size(); ++i)
      owner[i] = ref_value_to_control[i];
    buff_of_interest = curan::utilities::CaptureBuffer::make_shared(owner.data(), owner.size(), owner);
  } // memory owner is destroyed yet a copy of it is still inside the capture buffer, we are good
  ASSERT_EQ(comparator(ref_value_to_control, buff_of_interest), true) << "the buffers must be the same";
}
