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
	std::array<double, 2> variance_limit = { 1.0,20.0 };
	std::array<double, 2> disk_ratio_limit = { 0.1,10.0 };
	std::array<double, 2>  threshold_limit = { 50.0,150.0 };

	std::atomic<double> minimum_radius = 8;
	std::atomic<double> maximum_radius = 12.0;
	std::atomic<double> sweep_angle = 0.1;
	std::atomic<double> sigma_gradient = 10;
	std::atomic<double> variance = 10;
	std::atomic<double> disk_ratio = 1;
	std::atomic<double> threshold = 110;

};

struct ObservationEigenFormat {
	Eigen::Matrix<double, 4, 4> flange_data;
	Eigen::Matrix<double, 3, Eigen::Dynamic> segmented_wires;
};

struct PotentialObservationEigenFormat {
	std::optional<Eigen::Matrix<double, 4, 4>> flange_data;
	std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> segmented_wires;

	bool is_complete(ObservationEigenFormat& observation) {
		bool val = flange_data.has_value() && segmented_wires.has_value();
		if (!val)
			return val;
		observation.flange_data = *flange_data;
		observation.segmented_wires = *segmented_wires;
		flange_data = std::nullopt;
		segmented_wires = std::nullopt;
		return val;
	}

	void set_flange_data(Eigen::Matrix<double, 4, 4> flange_dat) {
		flange_data = flange_dat;
	}

	void set_segmented_wires(Eigen::Matrix<double, 3, Eigen::Dynamic> in_segmented_wires) {
		segmented_wires = in_segmented_wires;
	}
};

std::optional<Eigen::Matrix<double, 3, Eigen::Dynamic>> rearrange_wire_geometry(Eigen::Matrix<double, 3, Eigen::Dynamic> current, Eigen::Matrix<double, 3, Eigen::Dynamic> previous) {
	assert(current.cols()==previous.cols());
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distance_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(previous.cols(), previous.cols());
	for (size_t distance_row = 0; distance_row < previous.cols(); ++distance_row) {
		Eigen::Matrix<double, 3, 1> current_col = current.col(distance_row);
		for (size_t distance_col = 0; distance_col < previous.cols(); ++distance_col) {
			Eigen::Matrix<double, 3, 1> previous_col = previous.col(distance_col);
			distance_matrix(distance_row, distance_col) = (previous_col - current_col).norm();
		}
	}
}

struct ProcessingMessage {
	PotentialObservationEigenFormat observation_to_propagete;
	std::list<ObservationEigenFormat> list_of_recorded_points;

	std::shared_ptr<curan::ui::ImageDisplay> processed_viwer;
	std::shared_ptr<curan::ui::OpenIGTLinkViewer> open_viwer;
	std::shared_ptr<curan::utils::Flag> connection_status;
	std::shared_ptr<curan::ui::Button> button;
	std::shared_ptr<curan::ui::Button> button_start_collection;
	size_t number_of_circles = 3;
	asio::io_context io_context;
	ConfigurationData& configuration;
	
	std::atomic<bool> should_record = false;
	std::atomic<bool> show_circles = false;
	short port = 10000;

	ProcessingMessage(std::shared_ptr<curan::ui::ImageDisplay> in_processed_viwer,
		std::shared_ptr<curan::ui::OpenIGTLinkViewer> in_open_viwer,
		std::shared_ptr<curan::utils::Flag> flag, ConfigurationData& in_configuration) : connection_status{ flag }, processed_viwer{ in_processed_viwer }, open_viwer{ in_open_viwer }, configuration{ in_configuration }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif