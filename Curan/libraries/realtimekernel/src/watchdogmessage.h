
#include <cassert>
#include <memory>

struct watchdog_message
{	size_t counter;
	size_t sensors_receive_timestamp;
	size_t sensors_send_timestamp;
	size_t watchdog_command_timestamp;
	size_t watchdog_sensor_reqst_timestamp;
	size_t watchdog_sensor_receive_timestamp;
	size_t watchdog_client_reqst_timestamp;
	size_t watchdog_client_receive_timestamp;
	size_t client_receive_timestamp;
	int gps_reading_present;
	int image_reading_present;
};

struct watchdog_message_layout 
{	 size_t counter_address = 0;
	 size_t counter_size = 8;

	 size_t sensors_receive_timestamp_address = 8;
	 size_t sensors_receive_timestamp_size = 8;

	 size_t sensors_send_timestamp_address = 16;
	 size_t sensors_send_timestamp_size = 8;

	 size_t watchdog_command_timestamp_address = 24;
	 size_t watchdog_command_timestamp_size = 8;

	 size_t watchdog_sensor_reqst_timestamp_address = 32;
	 size_t watchdog_sensor_reqst_timestamp_size = 8;

	 size_t watchdog_sensor_receive_timestamp_address = 40;
	 size_t watchdog_sensor_receive_timestamp_size = 8;

	 size_t watchdog_client_reqst_timestamp_address = 48;
	 size_t watchdog_client_reqst_timestamp_size = 8;

	 size_t watchdog_client_receive_timestamp_address = 56;
	 size_t watchdog_client_receive_timestamp_size = 8;

	 size_t client_receive_timestamp_address = 64;
	 size_t client_receive_timestamp_size = 8;

	 size_t gps_reading_present_address = 72;
	 size_t gps_reading_present_size = 4;

	 size_t image_reading_present_address = 76;
	 size_t image_reading_present_size = 4;

};

void copy_from_watchdog_message_to_memory( unsigned char* memory , const watchdog_message & tmp)
{ 
	constexpr watchdog_message_layout mapping;

	std::memcpy( memory+mapping.counter_address , &tmp.counter , mapping.counter_size );

	std::memcpy( memory+mapping.sensors_receive_timestamp_address , &tmp.sensors_receive_timestamp , mapping.sensors_receive_timestamp_size );

	std::memcpy( memory+mapping.sensors_send_timestamp_address , &tmp.sensors_send_timestamp , mapping.sensors_send_timestamp_size );

	std::memcpy( memory+mapping.watchdog_command_timestamp_address , &tmp.watchdog_command_timestamp , mapping.watchdog_command_timestamp_size );

	std::memcpy( memory+mapping.watchdog_sensor_reqst_timestamp_address , &tmp.watchdog_sensor_reqst_timestamp , mapping.watchdog_sensor_reqst_timestamp_size );

	std::memcpy( memory+mapping.watchdog_sensor_receive_timestamp_address , &tmp.watchdog_sensor_receive_timestamp , mapping.watchdog_sensor_receive_timestamp_size );

	std::memcpy( memory+mapping.watchdog_client_reqst_timestamp_address , &tmp.watchdog_client_reqst_timestamp , mapping.watchdog_client_reqst_timestamp_size );

	std::memcpy( memory+mapping.watchdog_client_receive_timestamp_address , &tmp.watchdog_client_receive_timestamp , mapping.watchdog_client_receive_timestamp_size );

	std::memcpy( memory+mapping.client_receive_timestamp_address , &tmp.client_receive_timestamp , mapping.client_receive_timestamp_size );

	std::memcpy( memory+mapping.gps_reading_present_address , &tmp.gps_reading_present , mapping.gps_reading_present_size );

	std::memcpy( memory+mapping.image_reading_present_address , &tmp.image_reading_present , mapping.image_reading_present_size );

}


void copy_from_memory_to_watchdog_message( const unsigned char*  memory,watchdog_message & tmp)
{ 
	constexpr watchdog_message_layout mapping;

	std::memcpy( &tmp.counter,memory+mapping.counter_address , mapping.counter_size );

	std::memcpy( &tmp.sensors_receive_timestamp,memory+mapping.sensors_receive_timestamp_address , mapping.sensors_receive_timestamp_size );

	std::memcpy( &tmp.sensors_send_timestamp,memory+mapping.sensors_send_timestamp_address , mapping.sensors_send_timestamp_size );

	std::memcpy( &tmp.watchdog_command_timestamp,memory+mapping.watchdog_command_timestamp_address , mapping.watchdog_command_timestamp_size );

	std::memcpy( &tmp.watchdog_sensor_reqst_timestamp,memory+mapping.watchdog_sensor_reqst_timestamp_address , mapping.watchdog_sensor_reqst_timestamp_size );

	std::memcpy( &tmp.watchdog_sensor_receive_timestamp,memory+mapping.watchdog_sensor_receive_timestamp_address , mapping.watchdog_sensor_receive_timestamp_size );

	std::memcpy( &tmp.watchdog_client_reqst_timestamp,memory+mapping.watchdog_client_reqst_timestamp_address , mapping.watchdog_client_reqst_timestamp_size );

	std::memcpy( &tmp.watchdog_client_receive_timestamp,memory+mapping.watchdog_client_receive_timestamp_address , mapping.watchdog_client_receive_timestamp_size );

	std::memcpy( &tmp.client_receive_timestamp,memory+mapping.client_receive_timestamp_address , mapping.client_receive_timestamp_size );

	std::memcpy( &tmp.gps_reading_present,memory+mapping.gps_reading_present_address , mapping.gps_reading_present_size );

	std::memcpy( &tmp.image_reading_present,memory+mapping.image_reading_present_address , mapping.image_reading_present_size );

}