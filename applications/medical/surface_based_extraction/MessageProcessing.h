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
#include "utils/Reader.h"
#include "utils/Job.h"
#include "utils/TheadPool.h"
#include "imageprocessing/FilterAlgorithms.h"
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <iostream>
#include "userinterface/widgets/Plotter.h"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

struct ObservationEigenFormat {
	Eigen::Matrix4d pose;
    std::vector<std::pair<unsigned int, unsigned int>> segmented_points;
};

struct ProcessingMessage {
	std::list<ObservationEigenFormat> list_of_recorded_points;

	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::ui::ImageDisplay* filter_viwer = nullptr;

	curan::ui::ConfigDraw* config = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	curan::ui::Button* button_start_collection;
	asio::io_context io_context;

	Eigen::Matrix<double,4,4> calibration = Eigen::Matrix<double,4,4>::Identity();

	std::atomic<bool> snapshot = false;
    std::atomic<bool> record_poincloud = false;
    std::atomic<bool> store_to_file = false;

    //Segmentation parameters
	std::array<float, 2> min_coordx_limit = {0, 200};
    int min_coordx = 0;
	std::array<float, 2> max_coordx_limit = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
    int max_coordx = std::numeric_limits<float>::quiet_NaN();
	std::array<float, 2> lines_limit = {4, 100};
    int numLines = 50;
	
	std::shared_ptr<curan::utilities::ThreadPool> shared_pool = curan::utilities::ThreadPool::create(4);
	short port = 18944;

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,curan::ui::ImageDisplay* in_filter_viwer) : processed_viwer{ in_processed_viwer } , filter_viwer{in_filter_viwer}
	{
		nlohmann::json calibration_data;
        std::ifstream in(CURAN_COPIED_RESOURCE_PATH "/spatial_calibration.json");

        if (!in.is_open())
            throw std::runtime_error("failure to open configuration file");

        in >> calibration_data;
        std::string timestamp = calibration_data["timestamp"];
        std::string homogenenous_transformation = calibration_data["homogeneous_transformation"];
        double error = calibration_data["optimization_error"];
        std::printf("Using calibration with average error of : %f\n on the date ", error);
        std::cout << timestamp << std::endl;
        std::stringstream matrix_strm;
        matrix_strm << homogenenous_transformation;
        auto calibration_matrix = curan::utilities::convert_matrix(matrix_strm, ',');
        std::cout << "with the homogeneous matrix :\n"
                  << calibration_matrix << std::endl;

		calibration = calibration_matrix;
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif