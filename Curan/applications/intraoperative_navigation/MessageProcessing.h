#ifndef CURAN_MESSAGEPROCESSING_HEADER_FILE_
#define CURAN_MESSAGEPROCESSING_HEADER_FILE_

#include "LevelImplementation.h"
#include <vector>
#include <memory>

#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"

#include "utils/Flag.h"
#include "utils/TheadPool.h"

#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"



struct ProcessingMessage {
	curan::ui::ImageDisplay* processed_viwer = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	asio::io_context io_context;
    InputImageType::Pointer volume;

    Eigen::Matrix<double,3,1> target;
    Eigen::Matrix<double,3,1> entry_point;
    Eigen::Matrix<double,3,1> desired_rotation;

    std::shared_ptr<curan::utilities::ThreadPool> shared_pool;

    double image_size = 1.0;
    double image_spacing = 0.1;

	short port = 10000;

	ProcessingMessage(curan::ui::ImageDisplay* in_processed_viwer,InputImageType::Pointer in_volume);

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif