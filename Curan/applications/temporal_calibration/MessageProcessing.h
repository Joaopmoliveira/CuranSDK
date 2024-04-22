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

struct ObservationEigenFormat {
	Eigen::Vector3d flange_data;
	float video_signal;
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
	double aquisition_time = 10.0;
	//Edit to the actual fps of the frame grabber
	int fps = 25;
    //Segmentation parameters
    int min_coordx = 0+1;
    int max_coordx = 386-1;
    int numLines = 40;
    //RANSAC parameters
    int numIterations = 100;
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