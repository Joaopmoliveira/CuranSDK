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
	curan::ui::Button* connection_button;

	curan::ui::Button* record_world_button;
	curan::ui::Button* record_calib_button;

	asio::io_context io_context;
	std::shared_ptr<curan::communication::Client<curan::communication::protocols::igtlink>> client;
	ConfigurationData& configuration;
	std::atomic<bool> should_record_point_for_calibration = false;
	std::atomic<bool> should_record_point_from_world = false;
	short port = 10000;

	inline void push_calibration_point(const ObservationEigenFormat& calib){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points_for_calibration.push_back(calib);
	}

	inline void clear_calibration_points(){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points_for_calibration = std::list<ObservationEigenFormat>{};
	}

	inline void push_world_point(const ObservationEigenFormat& world){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points.push_back(world);
	}

	inline void clear_world_point(){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points = std::list<ObservationEigenFormat>{};
	}

	inline Eigen::Matrix<double,4,Eigen::Dynamic> world_points(){
		std::lock_guard<std::mutex> g{mut};
		Eigen::Matrix<double,4,Eigen::Dynamic> world_p = Eigen::Matrix<double,4,Eigen::Dynamic>::Ones(4,list_of_recorded_points.size());
		auto iterator = list_of_recorded_points.begin();
		for(size_t i = 0; i < list_of_recorded_points.size() ; ++i,++iterator)
			world_p.col(i) = (*iterator).flange_data.col(3);
		return world_p;
	}

	inline Eigen::Matrix<double,4,Eigen::Dynamic> calibration_points(){
		std::lock_guard<std::mutex> g{mut};
		Eigen::Matrix<double,4,Eigen::Dynamic> calib_p = Eigen::Matrix<double,4,Eigen::Dynamic>::Ones(4,list_of_recorded_points_for_calibration.size());
		auto iterator = list_of_recorded_points_for_calibration.begin();
		for(size_t i = 0; i < list_of_recorded_points_for_calibration.size() ; ++i,++iterator)
			calib_p.col(i) = (*iterator).flange_data.col(3);
		return calib_p;
	}

	inline bool calibrate_needle(){
		std::lock_guard<std::mutex> g{mut};
		if(list_of_recorded_points_for_calibration.size()==0)
			return false;
		Eigen::Matrix<double,4,Eigen::Dynamic> calib_p = Eigen::Matrix<double,4,Eigen::Dynamic>::Ones(4,list_of_recorded_points_for_calibration.size());
		auto iterator = list_of_recorded_points_for_calibration.begin();
		for(size_t i = 0; i < list_of_recorded_points_for_calibration.size() ; ++i,++iterator)
			calib_p.col(i) = (*iterator).flange_data.col(3);
		double conditioned = (calib_p.transpose()*calib_p).determinant();
		if(conditioned<1e-4)
			return false;
		Eigen::Matrix<double,4,1> solution = (calib_p.transpose()*calib_p).inverse()*calib_p.transpose()*calib_p.array().square().matrix();
		Eigen::Matrix<double,3,1> pivot_point = solution.block<3,1>(0,0);
		double radius = solution[4];
		if(radius<1e-4)
			return false;

		Eigen::Matrix<double,3,1> average_normal_vectors = Eigen::Matrix<double,3,1>::Zero();
		for(const auto& pose : calib_p.colwise())
			average_normal_vectors += (1.0/list_of_recorded_points_for_calibration.size())*(pivot_point-pose.block<3,1>(0,0));
		average_normal_vectors.normalize();
		Eigen::Matrix<double,4,4> calibrated_needle = Eigen::Matrix<double,4,4>::Identity();
		calibrated_needle.block<3,1>(0,3) = radius*calibrated_needle;
		needle_calibration = calibrated_needle;
		return true;
	}

	ProcessingMessage(ConfigurationData& config_data) : configuration{ config_data }
	{
	}

	bool process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val);

	void communicate();

	void attempt_stop();
};

#endif