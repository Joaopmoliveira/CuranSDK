#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"
#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	auto projeto = curan::utilities::ThreadPool::create(4);

	ConfigurationData data;
	data.shared_pool = projeto;
	std::cout << "the received port is: " << data.port << "\n";
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context),2200,1800 };
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(data,processing,resources);

	auto rec = viewer->get_size();
	page.propagate_size_change(rec);

	auto width = rec.width();
	auto height = rec.height();

	ConfigDraw config{&page};

	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		auto temp_height = pointer_to_surface->height();
		auto temp_width = pointer_to_surface->width();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (temp_height != height || temp_width != width) {
			rec = SkRect::MakeWH(temp_width, temp_height);
			page.propagate_size_change(rec);
		}
		page.draw(canvas);
		auto signals = viewer->process_pending_signals();
		if (!signals.empty())
			page.propagate_signal(signals.back(), &config);
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}
	processing->attempt_stop();
	std::cout << "trying to stop communication\n" << std::endl;

	std::printf("\nminimum_radius: (%f)\nmaximum_radius: (%f)\nsweep_angle: (%f)\nsigma_gradient: (%f)\nvariance: (%f)\nDiscRadiusRatio: (%f), Threshold (%f)\n", 
		data.minimum_radius.load(), data.maximum_radius.load(), data.sweep_angle.load(),data.sigma_gradient.load(), 
		data.variance.load(), data.disk_ratio.load(),data.threshold.load());

	if(processing->list_of_recorded_points.size()==0){
		std::cout << "no calibration data was recorded\n";
		return 1;
	}

	constexpr size_t number_of_strings = 3;
	constexpr size_t number_of_variables = 6 + 4 * number_of_strings;
	double variables[number_of_variables];

	curan::optim::WireData optimizationdata;
	optimizationdata.wire_data.reserve(number_of_strings * processing->list_of_recorded_points.size());
	
	for (const auto& f : processing->list_of_recorded_points) {
		curan::optim::Observation localobservation;
		localobservation.flange_configuration.values[0] = f.flange_data(0, 0);
		localobservation.flange_configuration.values[1] = f.flange_data(1, 0);
		localobservation.flange_configuration.values[2] = f.flange_data(2, 0);

		localobservation.flange_configuration.values[3] = f.flange_data(0, 1);
		localobservation.flange_configuration.values[4] = f.flange_data(1, 1);
		localobservation.flange_configuration.values[5] = f.flange_data(2, 1);

		localobservation.flange_configuration.values[6] = f.flange_data(0, 2);
		localobservation.flange_configuration.values[7] = f.flange_data(1, 2);
		localobservation.flange_configuration.values[8] = f.flange_data(2, 2);

		localobservation.flange_configuration.values[9] = f.flange_data(0, 3)*1e-3;
		localobservation.flange_configuration.values[10] = f.flange_data(1, 3)*1e-3;
		localobservation.flange_configuration.values[11] = f.flange_data(2, 3)*1e-3;

		size_t wire_number = 1;
		for (const auto& wire_observation : f.segmented_wires.colwise()) {
			localobservation.wire_number = wire_number;
			localobservation.wire_data.values[0] = wire_observation(0, 0)*0.0001852;
			localobservation.wire_data.values[1] = wire_observation(1, 0)*0.0001852;
			localobservation.wire_data.values[2] = wire_observation(2, 0)*0.0001852;
			optimizationdata.wire_data.push_back(localobservation);
			++wire_number;
		}
	}

	for (auto& val : variables)
		val = 0.0;

	ceres::Problem problem;
	for (const auto& data : optimizationdata.wire_data) {
		ceres::CostFunction* cost_function =
			new ceres::AutoDiffCostFunction<curan::optim::WiredResidual, 2, number_of_variables>(
				new curan::optim::WiredResidual(data));
		problem.AddResidualBlock(cost_function, nullptr, variables);
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 2500;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	
	std::cout << "Initial: \n";

	for (const auto& val : variables)
		std::cout << 0.0 << " , ";
	std::cout << "\n";
	std::stringstream optimized_values;
	std::cout << "Final: \n";
	for (const auto& val : variables){
		optimized_values << val << std::endl;
		std::cout << val << " , ";
	}
	
	auto return_current_time_and_date = [](){
	    auto now = std::chrono::system_clock::now();
    	auto in_time_t = std::chrono::system_clock::to_time_t(now);

    	std::stringstream ss;
    	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
   		return ss.str();
	};
	
	// Once the optimization is finished we need to print a json file with the correct configuration of the image transformation to the 
	// tracker transformation ()
	std::printf("\nRememeber that you always need to\nperform the temporal calibration before attempting the\nspacial calibration! Produced JSON file:\n");

	nlohmann::json calibration_data;
	calibration_data["timestamp"] = return_current_time_and_date();
	calibration_data["array_data"] = optimized_values.str();
	calibration_data["optimization_error"] = summary.final_cost;

	// write prettified JSON to another file
	std::ofstream o("optimization_result.json");
	o << calibration_data;
	std::cout << calibration_data << std::endl;
	return 0;
}