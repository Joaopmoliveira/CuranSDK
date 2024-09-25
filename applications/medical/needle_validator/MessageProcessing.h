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
	std::shared_ptr<curan::utilities::ThreadPool> shared_pool;
};

struct ObservationEigenFormat {
	Eigen::Matrix<double, 4, 4> flange_data;
};

struct ProcessingMessage {
private:
	std::list<ObservationEigenFormat> list_of_recorded_points;
	std::list<ObservationEigenFormat> list_of_recorded_points_for_calibration;

	std::mutex mut;
public:
	Eigen::Matrix<double,4,4> needle_calibration = Eigen::Matrix<double,4,4>::Identity();
	double calibration_error = 0.0;

	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	size_t number_of_circles = 3;
	size_t number_of_circles_plus_extra = 5;
	std::vector<SkColor> colors = { SK_ColorMAGENTA,SK_ColorBLUE,SK_ColorGREEN,SK_ColorYELLOW,SK_ColorRED};
	asio::io_context io_context;
	ConfigurationData& configuration;
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

	ProcessingMessage(ConfigurationData& config_data) : configuration{ config_data }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif