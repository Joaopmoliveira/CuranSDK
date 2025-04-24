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
#include <tuple>

struct ProcessingMessage {
private:
	std::mutex mut;
public:
	std::shared_ptr<curan::utilities::ThreadPool> pool = curan::utilities::ThreadPool::create(2);
	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::ui::OpenIGTLinkViewer* open_viwer = nullptr;
	curan::utilities::Flag connection_status;
	std::atomic<bool> record_images = false;
	curan::ui::Button* button;
	size_t number_of_circles = 3;
	std::vector<SkColor> colors = { SK_ColorMAGENTA,SK_ColorBLUE,SK_ColorGREEN,SK_ColorYELLOW,SK_ColorRED};
	asio::io_context io_context;
	Eigen::Matrix<double,4,4> calibration_transformation = Eigen::Matrix<double,4,4>::Identity();
	std::vector<std::tuple<Eigen::Matrix<double,3,3>,double>> line_parameterization;
	short port = 10000;


	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,curan::ui::OpenIGTLinkViewer* in_open_viwer);

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif