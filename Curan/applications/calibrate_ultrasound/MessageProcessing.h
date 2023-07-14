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

	std::array<double, 2> minimum_radius_limit = { 5.0,10.0 };
	std::array<double, 2> maximum_radius_limit = { 11.0,30.0 };
	std::array<double, 2> sweep_angle_limit = { 0.1,0.8 };
	std::array<double, 2> sigma_gradient_limit = { 1.0,20.0 };
	std::array<double, 2> variance_limit = { 1.0,5.0 };
	std::array<double, 2> disk_ratio_limit = { 0.1,10.0 };
	std::array<double, 2>  threshold_limit = { 50.0,150.0 };

	std::atomic<double> minimum_radius = 9.3;
	std::atomic<double> maximum_radius = 14.5;
	std::atomic<double> sweep_angle = 0.26;
	std::atomic<double> sigma_gradient = 12.755;
	std::atomic<double> variance = 5;
	std::atomic<double> disk_ratio = 2.888;
	std::atomic<double> threshold = 110.0;

	std::shared_ptr<curan::utilities::ThreadPool> shared_pool;
};

struct ObservationEigenFormat {
	Eigen::Matrix<double, 4, 4> flange_data;
	Eigen::Matrix<double, 3, Eigen::Dynamic> segmented_wires;
};


std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic>& current, Eigen::Matrix<double, 3, Eigen::Dynamic>& previous, double threshold);

struct ProcessingMessage {
	std::list<ObservationEigenFormat> list_of_recorded_points;

	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::ui::OpenIGTLinkViewer* open_viwer = nullptr;
	std::shared_ptr<curan::utilities::Flag> connection_status;
	curan::ui::Button* button;
	curan::ui::Button* button_start_collection;
	size_t number_of_circles = 3;
	std::vector<SkColor> colors = { SK_ColorMAGENTA,SK_ColorBLUE,SK_ColorGREEN };
	asio::io_context io_context;
	ConfigurationData& configuration;
	double threshold = 70.0;
	std::atomic<bool> should_record = false;
	std::atomic<bool> show_circles = false;
	short port = 10000;

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,
		curan::ui::OpenIGTLinkViewer* in_open_viwer,
		std::shared_ptr<curan::utilities::Flag> flag, ConfigurationData& in_configuration) : connection_status{ flag }, processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer }, configuration{ in_configuration }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif