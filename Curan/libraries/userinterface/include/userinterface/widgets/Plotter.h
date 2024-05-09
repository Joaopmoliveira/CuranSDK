#ifndef CURAN_PLOTTER_HEADER_FILE_
#define CURAN_PLOTTER_HEADER_FILE_

#include "Drawable.h"
#include "definitions/UIdefinitions.h"
#include "utils/Lockable.h"
#include <optional>
#include "IconResources.h"
#include "SignalProcessor.h"
#include "utils/CircularBuffer.h"
#include <vector>

namespace curan {
namespace ui {

class Plotter : public  curan::ui::Drawable , public curan::utilities::Lockable, public curan::ui::SignalProcessor<Plotter> {

    std::vector<curan::utilities::CircularBuffer<SkPoint>> buffers;
    std::vector<uint8_t> verbs;
    SkPath path;
    SkColor background = SK_ColorWHITE;
    

public:

inline SkColor get_background_color() {
	std::lock_guard<std::mutex> g{ get_mutex() };
	return background;
}

inline Plotter& set_background_color(SkColor color) {
	std::lock_guard<std::mutex> g{ get_mutex() };
	background = color;
    return *(this);
}


static std::unique_ptr<Plotter> make(const size_t& plotter_size, const size_t& number_of_buffers);

void compile() override;

~Plotter();

inline void append(const SkPoint& in,const size_t& index){
    assert(index < buffers.size());
    buffers[index].put(SkPoint{in});
}

curan::ui::drawablefunction draw() override;

curan::ui::callablefunction call() override;

private:

	Plotter(const size_t& buffer_size, const size_t& number_of_buffers);

	Plotter(const Plotter& other) = delete;

	bool compiled = false;
};

}
}

#endif