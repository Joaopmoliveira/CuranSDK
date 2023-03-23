#include "imageprocessing/FilterAlgorithms.h"

namespace curan {
namespace image {
namespace filtering {

void Filter::operator<< (std::shared_ptr<Implementation>) {

}

InternalImageType::Pointer Filter::get_input() {

}

InternalImageType::Pointer Filter::get_output(){

};

ThreholdFilter::ThreholdFilter(Info& info) {
	filter = FilterType::New();
	filter->SetOutsideValue(info.outside_value);
	filter->ThresholdOutside(info.lower_bound, info.upper_bound);
}

std::shared_ptr<ThreholdFilter> ThreholdFilter::make(Info& info) {
	return std::shared_ptr<ThreholdFilter>(new ThreholdFilter{info});
}

void ThreholdFilter::updateinfo(Info& info) {
	filter->SetOutsideValue(info.outside_value);
	filter->ThresholdOutside(info.lower_bound, info.upper_bound);
}

InternalImageType::Pointer ThreholdFilter::update() {

}
InternalImageType* ThreholdFilter::get_output() {

}
void ThreholdFilter::set_input(const InternalImageType*) {

}

CannyFilter::CannyFilter(Info& info) {

}

std::shared_ptr<CannyFilter> CannyFilter::make(Info& info) {
	return std::shared_ptr<CannyFilter>(new CannyFilter{ info });
}

void CannyFilter::updateinfo(Info& info) {

}

InternalImageType::Pointer CannyFilter::update() {

}

InternalImageType* CannyFilter::get_output() {

}

void CannyFilter::set_input(const InternalImageType*) {

}

BinarizeFilter::BinarizeFilter(Info& info) {

}

std::shared_ptr<BinarizeFilter>BinarizeFilter::make(Info& info) {
	return std::shared_ptr<BinarizeFilter>(new BinarizeFilter{ info });
}

void BinarizeFilter::updateinfo(Info& info) {

}

InternalImageType::Pointer BinarizeFilter::update() {

}

InternalImageType* BinarizeFilter::get_output() {

}

void BinarizeFilter::set_input(const InternalImageType*) {

}

}
}
}