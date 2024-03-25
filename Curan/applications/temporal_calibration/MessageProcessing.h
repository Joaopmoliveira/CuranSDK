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
	Eigen::Matrix<double, 4, 4> flange_data;
};

struct ProcessingMessage {
	std::list<ObservationEigenFormat> list_of_recorded_points;

	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	curan::ui::Button* button_start_collection;
	asio::io_context io_context;
	std::atomic<bool> show_line = false;
	std::atomic<bool> start_calibration = false;
	
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