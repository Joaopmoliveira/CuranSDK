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
#include "userinterface/widgets/Plotter.h"

struct ObservationEigenFormat {
	Eigen::Vector3d flange_data;
	float video_signal;
	long long time_stamp;
};

struct RANSAC_Output{
	std::vector<double> bestModelParams;
    std::vector<std::pair<unsigned int, unsigned int>> inlierPoints;
};

struct ProcessingMessage {
	std::list<ObservationEigenFormat> list_of_recorded_points;
	std::vector<double> projections;
	std::vector<double> normalized_video_signal;
	std::vector<double> normalized_position_signal;
	curan::ui::ImageDisplay* processed_viwer = nullptr;
	//curan::ui::Plotter* plot = nullptr;
	curan::ui::ConfigDraw* config = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	curan::ui::Button* button_start_collection;
	asio::io_context io_context;
	std::atomic<bool> show_line = false;
	std::atomic<bool> start_calibration = false;
	std::atomic<bool> calibration_finished = false;
	std::atomic<bool> show_calibration_lines = false;
	std::atomic<bool> show_pointstofit = false;
	double timer = 0.0;
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::vector<double> shifted_signal;

	//Recording parameters
	std::array<float, 2> initial_delay_limit = {0, 20};
	double initial_delay = 3.0;
	std::array<float, 2> aquisition_time_limit = {1, 60};
	double aquisition_time = 10.0;

    //Segmentation parameters
	std::array<float, 2> min_coordx_limit = {0, 200};
    int min_coordx = 0;
	std::array<float, 2> max_coordx_limit = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    int max_coordx = std::numeric_limits<float>::quiet_NaN();
	std::array<float, 2> lines_limit = {4, 100};
    int numLines = 50;

    //RANSAC parameters
	std::array<float, 2> numIterations_limit = {1, 1000};
    int numIterations = 500;
	std::array<float, 2> inlierThreshold_limit = {0, 10};
    double inlierThreshold = 1;

	float calibration_value =0;
	
	std::shared_ptr<curan::utilities::ThreadPool> shared_pool = curan::utilities::ThreadPool::create(4);
	short port = 18944;

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer) : processed_viwer{ in_processed_viwer }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif