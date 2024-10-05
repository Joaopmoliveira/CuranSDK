#ifndef CURAN_MESSAGEPROCESSING_HEADER_FILE_
#define CURAN_MESSAGEPROCESSING_HEADER_FILE_

#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/Slider.h"
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/Job.h"
#include "utils/TheadPool.h"
#include "imageprocessing/FilterAlgorithms.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <iostream>

struct ConfigurationData {
	int port = 18944;

	std::array<double, 2> minimum_radius_limit = { 3.0,5.0 };
	std::array<double, 2> maximum_radius_limit = { 5.0,30.0 };
	std::array<double, 2> sweep_angle_limit = { 0.1,0.99 };
	std::array<double, 2> sigma_gradient_limit = { 1.0,20.0 };
	std::array<double, 2> variance_limit = { 1.0,10.0 };
	std::array<double, 2> disk_ratio_limit = { 0.1,20.0 };
	std::array<double, 2> threshold_limit = { 50.0,200.0 };

	std::atomic<double> minimum_radius = 4.230001;
	std::atomic<double> maximum_radius = 5.000000;
	std::atomic<double> sweep_angle = 0.260000;
	std::atomic<double> sigma_gradient = 9.274997;
	std::atomic<double> variance = 5.0;
	std::atomic<double> disk_ratio = 11.146494;
	std::atomic<double> threshold = 125.0;

	std::shared_ptr<curan::utilities::ThreadPool> shared_pool;
};

struct ObservationEigenFormat {
	Eigen::Matrix<double, 4, 4> flange_data;
	Eigen::Matrix<double, 3, Eigen::Dynamic> segmented_wires;
};


std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic>& current, Eigen::Matrix<double, 3, Eigen::Dynamic>& previous, double threshold);

struct ProcessingMessage {
private:
	std::list<ObservationEigenFormat> list_of_recorded_points;
	std::mutex mut;
public:
	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::ui::OpenIGTLinkViewer* open_viwer = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	curan::ui::Button* button_start_collection;
	size_t number_of_circles = 3;
	size_t number_of_circles_plus_extra = 5;
	std::vector<SkColor> colors = { SK_ColorMAGENTA,SK_ColorBLUE,SK_ColorGREEN,SK_ColorYELLOW,SK_ColorRED};
	asio::io_context io_context;
	ConfigurationData& configuration;
	double threshold = 70.0;
	std::atomic<bool> should_record = false;
	std::atomic<bool> show_circles = false;
	short port = 10000;


	inline ObservationEigenFormat& list_back(){
		std::lock_guard<std::mutex> g{mut};
		return list_of_recorded_points.back();
	}

	inline void list_push_back(const ObservationEigenFormat& format){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points.push_back(format);
	}

	inline size_t list_size(){
		std::lock_guard<std::mutex> g{mut};
		return list_of_recorded_points.size();
	}
 
	inline std::list<ObservationEigenFormat> list_deep_copy(){
		std::list<ObservationEigenFormat> copy;
		std::lock_guard<std::mutex> g{mut};
		std::copy(std::begin(list_of_recorded_points),std::end(list_of_recorded_points),std::back_inserter(copy));
		return copy;
	}

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,
		curan::ui::OpenIGTLinkViewer* in_open_viwer, ConfigurationData& in_configuration) : processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer }, configuration{ in_configuration }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif