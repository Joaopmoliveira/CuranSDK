#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"
#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"

#include <random>

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	auto shared_pool = curan::utilities::ThreadPool::create(4);

	ConfigurationData data;
	data.shared_pool = shared_pool;
	std::cout << "the received port is: " << data.port << "\n";
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context)};
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(data,processing,resources);

	page.update_page(viewer.get());

	ConfigDraw config{&page};

	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (viewer->was_updated()) {
	    	page.update_page(viewer.get());
			viewer->update_processed();
		}
		page.draw(canvas);
		auto signals = viewer->process_pending_signals();
		std::printf("%llu\n",signals.size());
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

	constexpr size_t number_of_initial_guesses = 20;
	struct initial_guess_and_cost{
		double final_cost;
		std::vector<double> initial_guess;
		ceres::Solver::Summary summary;
	};
	std::array<initial_guess_and_cost,number_of_initial_guesses> random_solutions_to_test;

	constexpr size_t number_of_strings = 3;
	constexpr size_t number_of_variables = 6 + 4 * number_of_strings;

	curan::optim::WireData optimizationdata;
	optimizationdata.wire_data.reserve(number_of_strings * processing->list_of_recorded_points.size());

	nlohmann::json flange_data_to_matlab;
	flange_data_to_matlab["number_of_observations"] = processing->list_of_recorded_points.size();
	flange_data_to_matlab["number_of_wires"] = number_of_strings;
	
	size_t counter = 0;
	for (const auto& f : processing->list_of_recorded_points) {
		nlohmann::json recording;
		std::stringstream ss;
		ss << f.flange_data;
		recording["flange_data"] = ss.str();
		ss.str("");
		ss << f.segmented_wires;
		recording["wire_config"] = ss.str();
		std::string val = "recording_" + std::to_string(counter);
		flange_data_to_matlab[val] = recording;
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

		size_t wire_number = 0;
		for (const auto& wire_observation : f.segmented_wires.colwise()) {
			localobservation.wire_number = wire_number;
			localobservation.wire_data.values[0] = wire_observation(0, 0)*0.0001852;
			localobservation.wire_data.values[1] = wire_observation(1, 0)*0.0001852;
			localobservation.wire_data.values[2] = 0.0;
			optimizationdata.wire_data.push_back(localobservation);
			++wire_number;
		}
		++counter;
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 2500;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

   double lower_bound = -10.0;
   double upper_bound = +10.0;
   std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
   std::default_random_engine re;

	for(auto& potential_solution : random_solutions_to_test){
		potential_solution.initial_guess.resize(number_of_variables);
		for (auto& val : potential_solution.initial_guess)
			val = unif(re);

		ceres::Problem problem;
		for (const auto& data : optimizationdata.wire_data) {
			ceres::CostFunction* cost_function =
				new ceres::AutoDiffCostFunction<curan::optim::WiredResidual, 2, number_of_variables>(
					new curan::optim::WiredResidual(data));
			problem.AddResidualBlock(cost_function, nullptr, potential_solution.initial_guess.data());
		}

		ceres::Solve(options, &problem, &potential_solution.summary);
		potential_solution.final_cost = potential_solution.summary.final_cost;
	}

	size_t minimum = 0 ;
	size_t i = 0 ;
	double minimum_value = 1e10;
	assert(random_solutions_to_test.size()>0);
	for(const auto& potential_solution : random_solutions_to_test){
		if(potential_solution.final_cost< minimum_value){
			minimum = i;
			minimum_value = potential_solution.final_cost;
		}
		++i;
	}

	auto best_run = random_solutions_to_test[minimum];
	std::cout << best_run.summary.BriefReport() << "\n";

	double t1 = cos(best_run.initial_guess[2]);
    double t2 = sin(best_run.initial_guess[2]);
    double t3 = cos(best_run.initial_guess[1]);
	double t4 = sin(best_run.initial_guess[1]);
    double t5 = cos(best_run.initial_guess[0]);
    double t6 = sin(best_run.initial_guess[0]);

	Eigen::Matrix<double,4,4> transformation_matrix = Eigen::Matrix<double,4,4>::Identity();
	transformation_matrix(0,0) = t1 * t3;
	transformation_matrix(1,0) = t2 * t3;
	transformation_matrix(2,0) = -t4;

	transformation_matrix(0,1) = t1 * t4 * t6 - t2 * t5;
	transformation_matrix(1,1) = t1 * t5 + t2 * t4 * t6;
	transformation_matrix(2,1) = t3 * t6;

	transformation_matrix(0,2) = t2 * t6 + t1 * t4 * t5;
	transformation_matrix(1,2) = t2 * t4 * t5 - t1 * t6;
	transformation_matrix(2,2) = t3 * t5;

	transformation_matrix(0,3) = best_run.initial_guess[3];
	transformation_matrix(1,3) = best_run.initial_guess[4];
	transformation_matrix(2,3) = best_run.initial_guess[5];

	std::cout << "Initial: \n" << Eigen::Matrix<double,4,4>::Identity() << std::endl;
	std::cout << "Final: \n" << transformation_matrix << std::endl;
	std::cout << "(all units in meters)" << std::endl; 

	auto return_current_time_and_date = [](){
	    auto now = std::chrono::system_clock::now();
    	auto in_time_t = std::chrono::system_clock::to_time_t(now);

    	std::stringstream ss;
    	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
   		return ss.str();
	};

	Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");
	std::stringstream optimized_values;
	optimized_values << transformation_matrix.format(CleanFmt) << std::endl;
	
	// Once the optimization is finished we need to print a json file with the correct configuration of the image transformation to the 
	// tracker transformation ()
	std::printf("\nRememeber that you always need to\nperform the temporal calibration before attempting the\nspacial calibration! Produced JSON file:\n");

	nlohmann::json calibration_data;
	calibration_data["timestamp"] = return_current_time_and_date();
	calibration_data["homogeneous_transformation"] = optimized_values.str();
	calibration_data["optimization_error"] = best_run.summary.final_cost;

	// write prettified JSON to another file
	std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/optimization_result.json");
	o << calibration_data;
	std::cout << calibration_data << std::endl;

	std::ofstream data_to_matlab(CURAN_COPIED_RESOURCE_PATH"/data_to_matlab.json");
	data_to_matlab << flange_data_to_matlab << std::endl;
	return 0;
}