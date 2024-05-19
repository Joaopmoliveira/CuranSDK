
#include <cassert>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <memory>
struct gps_reading
{	int counter;
	double latitude;
	double longitude;
	double height;
	double velocity[3];
	double acceleration[3];
	double gforce;
	double orientation[3];
	double angular_velocity[3];
	double standard_deviation[3];
};

struct gps_reading_layout 
{	 size_t counter_address = 0;
	 size_t counter_size = 4;

	 size_t latitude_address = 4;
	 size_t latitude_size = 8;

	 size_t longitude_address = 12;
	 size_t longitude_size = 8;

	 size_t height_address = 20;
	 size_t height_size = 8;

	 size_t velocity_address = 28;
	 size_t velocity_size = 24;

	 size_t acceleration_address = 52;
	 size_t acceleration_size = 24;

	 size_t gforce_address = 76;
	 size_t gforce_size = 8;

	 size_t orientation_address = 84;
	 size_t orientation_size = 24;

	 size_t angular_velocity_address = 108;
	 size_t angular_velocity_size = 24;

	 size_t standard_deviation_address = 132;
	 size_t standard_deviation_size = 24;

};

void copy_from_gps_reading_to_shared_memory( unsigned char* memory , const gps_reading & tmp)
{ 
	constexpr gps_reading_layout mapping;

	std::memcpy( memory+mapping.counter_address , &tmp.counter , mapping.counter_size );

	std::memcpy( memory+mapping.latitude_address , &tmp.latitude , mapping.latitude_size );

	std::memcpy( memory+mapping.longitude_address , &tmp.longitude , mapping.longitude_size );

	std::memcpy( memory+mapping.height_address , &tmp.height , mapping.height_size );

	std::memcpy( memory+mapping.velocity_address , &tmp.velocity , mapping.velocity_size );

	std::memcpy( memory+mapping.acceleration_address , &tmp.acceleration , mapping.acceleration_size );

	std::memcpy( memory+mapping.gforce_address , &tmp.gforce , mapping.gforce_size );

	std::memcpy( memory+mapping.orientation_address , &tmp.orientation , mapping.orientation_size );

	std::memcpy( memory+mapping.angular_velocity_address , &tmp.angular_velocity , mapping.angular_velocity_size );

	std::memcpy( memory+mapping.standard_deviation_address , &tmp.standard_deviation , mapping.standard_deviation_size );

}


void copy_from_shared_memory_to_gps_reading( const unsigned char*  memory,gps_reading & tmp)
{ 
	constexpr gps_reading_layout mapping;

	std::memcpy( &tmp.counter,memory+mapping.counter_address , mapping.counter_size );

	std::memcpy( &tmp.latitude,memory+mapping.latitude_address , mapping.latitude_size );

	std::memcpy( &tmp.longitude,memory+mapping.longitude_address , mapping.longitude_size );

	std::memcpy( &tmp.height,memory+mapping.height_address , mapping.height_size );

	std::memcpy( &tmp.velocity,memory+mapping.velocity_address , mapping.velocity_size );

	std::memcpy( &tmp.acceleration,memory+mapping.acceleration_address , mapping.acceleration_size );

	std::memcpy( &tmp.gforce,memory+mapping.gforce_address , mapping.gforce_size );

	std::memcpy( &tmp.orientation,memory+mapping.orientation_address , mapping.orientation_size );

	std::memcpy( &tmp.angular_velocity,memory+mapping.angular_velocity_address , mapping.angular_velocity_size );

	std::memcpy( &tmp.standard_deviation,memory+mapping.standard_deviation_address , mapping.standard_deviation_size );

}
struct grayscale_image_1
{	int counter;
	unsigned char* data = nullptr;
};

struct grayscale_image_1_layout 
{	 size_t counter_address = 156;
	 size_t counter_size = 4;

	 size_t data_address = 160;
	 size_t data_size = 5880000;

};

void copy_from_grayscale_image_1_to_shared_memory( unsigned char* memory , const grayscale_image_1 & tmp)
{ 
	constexpr grayscale_image_1_layout mapping;

	std::memcpy( memory+mapping.counter_address , &tmp.counter , mapping.counter_size );

	assert( tmp.data!=nullptr);
	std::memcpy( memory+mapping.data_address , tmp.data , mapping.data_size );

}


void copy_from_shared_memory_to_grayscale_image_1( const unsigned char*  memory,grayscale_image_1 & tmp)
{ 
	constexpr grayscale_image_1_layout mapping;

	std::memcpy( &tmp.counter,memory+mapping.counter_address , mapping.counter_size );

	assert( tmp.data!=nullptr);
	std::memcpy( tmp.data,memory+mapping.data_address , mapping.data_size );

}
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
		shm.truncate(5880160);
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
	inline size_t size(){
		return 5880160;
	}};

