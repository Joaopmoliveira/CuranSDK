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
	int port = 30008;
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

	std::optional<std::tuple<Eigen::Matrix<double,4,4>,double>> registration_solution;

	Eigen::Matrix<double,3,Eigen::Dynamic> landmarks;

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

	inline size_t size_calibration_points(){
		std::lock_guard<std::mutex> g{mut};
		return list_of_recorded_points_for_calibration.size();
	}

	inline void push_world_point(const ObservationEigenFormat& world){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points.push_back(world);
	}

	inline void clear_world_point(){
		std::lock_guard<std::mutex> g{mut};
		list_of_recorded_points = std::list<ObservationEigenFormat>{};
	}

	inline std::optional<Eigen::Matrix<double,3,Eigen::Dynamic>> world_points(){
		std::lock_guard<std::mutex> g{mut};
		if(list_of_recorded_points.size()==0)
			return std::nullopt;
		Eigen::Matrix<double,3,Eigen::Dynamic> world_p = Eigen::Matrix<double,3,Eigen::Dynamic>::Ones(4,list_of_recorded_points.size());
		auto iterator = list_of_recorded_points.begin();
		for(size_t i = 0; i < list_of_recorded_points.size() ; ++i,++iterator)
			world_p.col(i) = (*iterator).flange_data.col(3).block<3,1>(0,0);
		return world_p;
	}

	[[nodiscard]] inline bool calibrate_needle(){
		std::lock_guard<std::mutex> g{mut};
		if(list_of_recorded_points_for_calibration.size()==0)
			return false;
		Eigen::Matrix<double,Eigen::Dynamic,4> calib_p = Eigen::Matrix<double,Eigen::Dynamic,4>::Ones(list_of_recorded_points_for_calibration.size(),4);
		auto iterator = list_of_recorded_points_for_calibration.begin();
		for(size_t i = 0; i < list_of_recorded_points_for_calibration.size() ; ++i,++iterator)
			calib_p.row(i) = (*iterator).flange_data.col(3).transpose();
		double conditioned = (calib_p.transpose()*calib_p).determinant();
		if(std::abs(conditioned)<1e-4)
			return false;
		std::cout << (calib_p.array().square().rowwise().sum()-1.0).matrix() << std::endl;

		Eigen::Matrix<double,4,1> solution = (calib_p.transpose()*calib_p).inverse()*(calib_p.transpose())*(calib_p.array().square().rowwise().sum()-1.0).matrix();
		Eigen::Matrix<double,3,1> pivot_point = solution.block<3,1>(0,0)*0.5;


		double radius = std::sqrt(solution[3]+pivot_point.transpose()*pivot_point);
		if(radius<1e-4)
			return false;


		Eigen::Matrix<double,3,1> average_normal_vectors = Eigen::Matrix<double,3,1>::Zero();
		iterator = list_of_recorded_points_for_calibration.begin();
		for(size_t i = 0; i < list_of_recorded_points_for_calibration.size() ; ++i,++iterator){
			Eigen::Matrix<double,4,1> pivot_point_homogenenous = Eigen::Matrix<double,4,1>::Ones();
			pivot_point_homogenenous.block<3,1>(0,0) = pivot_point;
			Eigen::Matrix<double,3,1> to_normalize = ((((*iterator).flange_data).inverse()*pivot_point_homogenenous)).block<3,1>(0,0);
			to_normalize.normalize();
			average_normal_vectors += (1.0/list_of_recorded_points_for_calibration.size())*to_normalize;
		}
			
		average_normal_vectors.normalize();

		Eigen::Matrix<double,4,4> calibrated_needle = Eigen::Matrix<double,4,4>::Identity();
		calibrated_needle.block<3,1>(0,3) = radius*average_normal_vectors;
		needle_calibration = calibrated_needle;


		calibration_error = 0.0;
		iterator = list_of_recorded_points_for_calibration.begin();
		for(size_t i = 0; i < list_of_recorded_points_for_calibration.size() ; ++i,++iterator){
			Eigen::Matrix<double,4,1> pivot_point_homogenenous = Eigen::Matrix<double,4,1>::Ones();
			pivot_point_homogenenous.block<3,1>(0,0) = pivot_point;
			auto supposed_pivot_point = (*iterator).flange_data*needle_calibration;
			calibration_error += (1.0/list_of_recorded_points_for_calibration.size())*(supposed_pivot_point.col(3)-pivot_point_homogenenous).norm();
		}

		std::cout << "calibration error: " << calibration_error << std::endl;
		std::cout << "calibration matrix: \n" << needle_calibration << std::endl;

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