#include "imageprocessing/FilterAlgorithms.h"

namespace curan {
namespace image {
namespace filtering {

ThreholdFilter::ThreholdFilter(Info& info) {
	
}

std::shared_ptr<ThreholdFilter> ThreholdFilter::make(Info& info) {
	return std::shared_ptr<ThreholdFilter>(new ThreholdFilter{ info });
}

void ThreholdFilter::submit_image(Image input) {

}

void ThreholdFilter::updateinfo(Info& info) {

}

bool ThreholdFilter::update() {
	return true;
}

CannyFilter::CannyFilter(Info& info) {

}

std::shared_ptr<CannyFilter> CannyFilter::make(Info& info) {
	return std::shared_ptr<CannyFilter>(new CannyFilter{ info });
}

void CannyFilter::submit_image(Image input) {

}

void CannyFilter::updateinfo(Info& info) {

}

bool CannyFilter::update() {
	return true;
}

BinarizeFilter::BinarizeFilter(Info& info) {

}

std::shared_ptr<BinarizeFilter> BinarizeFilter::make(Info& info) {
	return std::shared_ptr<BinarizeFilter>(new BinarizeFilter{info});
}

void BinarizeFilter::submit_image(Image input) {

}

void BinarizeFilter::updateinfo(Info& info) {

}

bool BinarizeFilter::update() {
	return true;
}

}
}
}