#ifndef CURAN_APPLICATION_DATA_HEADER_FILE_
#define CURAN_APPLICATION_DATA_HEADER_FILE_

#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/Container.h"
#include "userinterface/widgets/OpenIGTLinkViewer.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/Loader.h"
#include "userinterface/widgets/TaskManager.h"
#include "utils/TheadPool.h"
#include "utils/Logger.h"
#include "utils/FileStructures.h"
#include "utils/Overloading.h"
#include <variant>
#include <csignal>

#include <iostream>
#include <boost/process.hpp>
#include <boost/asio/read_until.hpp>

constexpr auto waiting_color_active = SkColorSetARGB(70, 0, 255, 0);
constexpr auto waiting_color_inactive = SkColorSetARGB(70, 255, 0, 0);

class Application;

class PendinAsyncData : public std::enable_shared_from_this<PendinAsyncData>
{
	static size_t identifier;
	boost::asio::io_context &asio_ctx;

	std::string executable_name = ">>"; 
    bool plus_lauched = false;

	boost::process::child child_process;
	boost::process::group grou;
	boost::process::async_pipe child_out;
	boost::asio::streambuf child_buf;

#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
    boost::process::child plus_process;
	boost::process::group plus_grou;
	boost::asio::streambuf plus_buf;
	boost::process::async_pipe plus_out;
#endif

	Application *parent = nullptr;
    bool terminated_called = false;

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void post_async_plus_read();
#endif
	void post_async_read();

	void async_launch_child(const std::string &executable);

#ifdef CURAN_PLUS_EXECUTABLE_PATH
	void async_launch_plus();																		
#endif

	
	bool async_launch_all(const std::string &executable, bool all = true);
	struct Private
	{
		explicit Private() = default;
	};

public:
#ifdef CURAN_PLUS_EXECUTABLE_PATH // conditionally compile code with plus process lauching mechanics
	PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all = true);
#else
	PendinAsyncData(Private, boost::asio::io_context &in_asio_ctx, const std::string &executable, Application *in_parent, bool all = true);
#endif

	~PendinAsyncData();

	inline std::shared_ptr<PendinAsyncData> getptr()
	{
		return shared_from_this();
	}

	static std::shared_ptr<PendinAsyncData> make(boost::asio::io_context &asio_ctx, const std::string &executable, Application *in_parent, bool all = true);

	void async_terminate_all();
};

std::unique_ptr<curan::ui::Overlay> warning_overlay(const std::string &warning,curan::ui::IconResources& resources);

class Application
{
	curan::ui::Button *ptr_button1 = nullptr;
	curan::ui::Button *ptr_button2 = nullptr;
	curan::ui::Button *ptr_button3 = nullptr;
	curan::ui::Button *ptr_button4 = nullptr;
	curan::ui::Button *ptr_button5 = nullptr;
	curan::ui::Button *ptr_button6 = nullptr;
	
	std::shared_ptr<PendinAsyncData> pending_task = nullptr;

	std::atomic<int> pending_applications = 0;

	boost::asio::io_context asio_ctx;
	boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work;
	std::shared_ptr<curan::utilities::ThreadPool> pool;

	curan::ui::IconResources &resources;

	std::unique_ptr<curan::ui::Page> page = nullptr;

public:

	inline void register_launch(){
		++pending_applications;
	}

	inline void register_termination(){
		--pending_applications;
		if(pending_applications<0){
			throw std::runtime_error("failure due to more terminated applications than possible");
		}
	}

    inline int pending(){
        return pending_applications;
    }

	Application(curan::ui::IconResources &in_resources);

	~Application();

	void reset_work();

	bool terminated();

	std::unique_ptr<curan::ui::Container> create_main_widget_container();

	bool launch_all(const std::string &executable, bool all = true);

	void terminate_all();

	inline curan::ui::Page *get_page()
	{
		return page.get();
	}

	void warn_terminate_all();

	void termination_widget_logic();
};

#endif
