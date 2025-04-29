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
#include "communication/ProtoFRI.h"

struct ProcessingMessageInfo{
	curan::ui::ImageDisplay * f_in_processed_viwer;
	InputImageType::Pointer f_in_volume;
	Eigen::Matrix<double, 3, 1> f_target;
	Eigen::Matrix<double, 3, 1> f_entry_point;
	Eigen::Matrix<double, 3, 3> f_desired_rotation;

	Eigen::Matrix<double, 4, 4> f_registration;
	Eigen::Matrix<double, 4, 4> f_needle_calibration;
};

struct ProcessingMessage
{
	curan::ui::ImageDisplay *processed_viwer = nullptr;
	std::atomic<bool> connection_status = false;
	curan::ui::Button *button;
	asio::io_context io_context;
	InputImageType::Pointer volume;

	const Eigen::Matrix<double, 3, 1> target;
	const Eigen::Matrix<double, 3, 1> entry_point;
	const Eigen::Matrix<double, 3, 3> desired_rotation;

	const Eigen::Matrix<double, 4, 4> registration;
	const Eigen::Matrix<double, 4, 4> needle_calibration;

	std::shared_ptr<curan::utilities::ThreadPool> shared_pool;

	double image_size = 1.0;
	double image_spacing = 0.1;

	short port = 10000;

	ProcessingMessage(const ProcessingMessageInfo& info);

	bool process_joint_message(const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message);

	void communicate();

	void attempt_stop();
};

#endif