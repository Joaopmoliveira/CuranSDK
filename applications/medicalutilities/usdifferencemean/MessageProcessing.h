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
#include "itkMeanImageFilter.h"

struct ConfigurationData {
	int port = 18944;
	std::shared_ptr<curan::utilities::ThreadPool> shared_pool;
};

std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic>& current, Eigen::Matrix<double, 3, Eigen::Dynamic>& previous, double threshold);

struct ProcessingMessage {
	using ImageType = itk::Image<unsigned char, 3>;
	using DiferenceImageType = itk::Image<unsigned char, 3>;

	std::list<ImageType::Pointer> images_recorded; 
	std::list<std::vector<double>> regions_recorded; 

	curan::ui::ConfigDraw* config_draw;

	std::optional<ImageType::Pointer> mean_image_computed;
	std::optional<std::vector<double>> average_inactive_differences;

	curan::ui::ImageDisplay* difference_image = nullptr;
	curan::ui::ImageDisplay* accepted_region_image = nullptr;
	curan::utilities::Flag connection_status;
	curan::ui::Button* button;
	asio::io_context io_context;
	ConfigurationData& configuration;
	short port = 10000;

	ProcessingMessage(curan::ui::ImageDisplay* in_difference_image,curan::ui::ImageDisplay* in_accepted_region_image, ConfigurationData& in_configuration, curan::ui::ConfigDraw* in_config_draw) : difference_image{ in_difference_image }, accepted_region_image{in_accepted_region_image}, configuration{ in_configuration }, config_draw{in_config_draw}
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif