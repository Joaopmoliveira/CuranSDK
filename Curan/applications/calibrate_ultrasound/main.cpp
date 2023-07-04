#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	curan::utilities::initialize_thread_pool(10);

	ConfigurationData data;
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
	std::cout << "trying to stop everything" << std::endl;

	int tasks_n = 0;
	int tasks_queue = 0;
	curan::utilities::pool->get_number_tasks(tasks_n, tasks_queue);
	std::cout << "Number of tasks executing: " << tasks_n << " number of tasks in queue" << tasks_queue << "\n";
	curan::utilities::terminate_thread_pool();
	std::cout << "Number of frame recordings: " << processing->list_of_recorded_points.size() << "\n";
	std::cout << "Received spacing: \n";

	return 0;

	constexpr size_t number_of_strings = 3;
	constexpr size_t number_of_variables = 6 + 4 * number_of_strings;
	double variables[number_of_variables];

	curan::optim::WireData optimizationdata;
	optimizationdata.wire_data.reserve(number_of_strings * processing->list_of_recorded_points.size());
	int counter_f = 1;
	for (const auto& f : processing->list_of_recorded_points) {
		size_t wire_number = 1;
		curan::optim::Observation localobservation;
		Eigen::Matrix<double, 4, 4> temp = Eigen::Matrix<double, 4, 4>::Zero();

		localobservation.flange_configuration.values[0] = temp(0, 0);
		localobservation.flange_configuration.values[1] = temp(1, 0);
		localobservation.flange_configuration.values[2] = temp(2, 0);

		localobservation.flange_configuration.values[3] = temp(0, 1);
		localobservation.flange_configuration.values[4] = temp(1, 1);
		localobservation.flange_configuration.values[5] = temp(2, 1);

		localobservation.flange_configuration.values[6] = temp(0, 2);
		localobservation.flange_configuration.values[7] = temp(1, 2);
		localobservation.flange_configuration.values[8] = temp(2, 2);

		localobservation.flange_configuration.values[9] = temp(0, 3);
		localobservation.flange_configuration.values[10] = temp(1, 3);
		localobservation.flange_configuration.values[11] = temp(2, 3);

		localobservation.wire_number = wire_number;

		Eigen::Matrix<double, 3, 1> temp2 = Eigen::Matrix<double, 3, 1>::Zero();

		localobservation.wire_data.values[0] = temp2(0, 0);
		localobservation.wire_data.values[1] = temp2(1, 0);
		localobservation.wire_data.values[2] = temp2(2, 0);
		optimizationdata.wire_data.push_back(localobservation);
		std::cout << "slice : " << counter_f << "\n";
		std::cout << "->transformation : " << f.flange_data << "\n";
		for (const auto& p : f.segmented_wires.colwise()) {
			std::cout << "	point(" << wire_number << ") -> (" << p(0,0) << ", " << p(1,0) << ")\n";
			++wire_number;
		}

		++counter_f;
	}
	std::cout << "\n";


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
	options.max_num_iterations = 250;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	std::cout << "Initial: \n";

	for (const auto& val : variables)
		std::cout << 0.0 << " , ";
	std::cout << "\n";
	std::cout << "Final: \n";
	for (const auto& val : variables)
		std::cout << val << " , ";
	return 0;
}