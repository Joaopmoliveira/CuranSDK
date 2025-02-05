#ifndef CURAN_LIGHT_PAGE_HEADER_FILE_
#define CURAN_LIGHT_PAGE_HEADER_FILE_

#include "Container.h"

namespace curan {
namespace ui {

/*

*/

using post_signal_callback = std::function<void(Signal sig, bool page_interaction, ConfigDraw* config)>;

struct compilation_results {
	std::vector<drawablefunction> callable_draw;
	std::vector<callablefunction> callable_signal;
};

class LightWeightPage {
public:

	static std::unique_ptr<LightWeightPage> make(std::unique_ptr<Container> contained, SkColor backgroundcolor, bool tight = false);

	~LightWeightPage();

	LightWeightPage& draw(SkCanvas* canvas);

	inline bool terminated(){
		return request_to_terminate.load();
	}

	inline void terminate(bool signal_to_post){
		request_to_terminate.store(signal_to_post);
	}

	bool propagate_signal(Signal sig, ConfigDraw* config);

	LightWeightPage& propagate_size_change(const SkRect& new_size);

	LightWeightPage& set_post_signal(post_signal_callback call);

	inline SkRect minimum_size(){
		return cached_minimum_size;
	}

	inline LightWeightPage& set_dirtyness(bool var) {
		is_dirty = var;
        return *(this);
	}

private:

	LightWeightPage(std::unique_ptr<Container> contained, SkColor backgroundcolor, SkRect computed_minimum_size,bool tight = false);

	std::unique_ptr<Container> scene;
	std::atomic<bool> is_dirty = false;
	compilation_results compiled_scene;
	SkColor backgroundcolor = SK_ColorWHITE;
	post_signal_callback post_signal_processing;
	SkRect cached_minimum_size;
	SkPaint paint;
	bool is_tight;
	std::atomic<bool> request_to_terminate = false;

};

}
}

#endif