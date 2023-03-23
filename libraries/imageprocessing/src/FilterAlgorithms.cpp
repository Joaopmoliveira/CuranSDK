#include "imageprocessing/FilterAlgorithms.h"

namespace curan {
namespace image {
namespace filtering {

void Filter::operator << (std::shared_ptr<Implementation> sequencial_filter) {
	if (filters.size() == 0) {
		sequencial_filter->set_input(input);
		filters.push_back(sequencial_filter);
		return;
	}
	auto temp = filters.back();
	sequencial_filter->set_input(temp->get_output());
	filters.push_back(sequencial_filter);
	return;
}

Internal2DImageType::Pointer Filter::get_input() {
	return input;
}

Internal2DImageType::Pointer Filter::get_output(){
	if (filters.size() == 0)
		return nullptr;
	auto temp = filters.back();
	return temp->update_and_return_out();

};

ThreholdFilter::ThreholdFilter(Info& info) {
	filter = FilterType::New();
	updateinfo(info);
}

std::shared_ptr<ThreholdFilter> ThreholdFilter::make(Info& info) {
	return std::shared_ptr<ThreholdFilter>(new ThreholdFilter{info});
}

void ThreholdFilter::updateinfo(Info& info) {
	filter->SetOutsideValue(info.outside_value);
	filter->ThresholdOutside(info.lower_bound, info.upper_bound);
}

Internal2DImageType* ThreholdFilter::get_output() {
	return filter->GetOutput();
}
void ThreholdFilter::set_input(const Internal2DImageType* val_in) {
	filter->SetInput(val_in);
}

Internal2DImageType::Pointer ThreholdFilter::update_and_return_out() {
	try
	{
		filter->Update();
	}
	catch (const itk::ExceptionObject& err)
	{
		return nullptr;
	}
	return filter->GetOutput();
}

CannyFilter::CannyFilter(Info& info) {
	cast_to_real = CastToRealFilterType::New();
	canny_filter = CannyFilterType::New();
	cast_to_char = RescaleFilterType::New();

	updateinfo(info);
}

std::shared_ptr<CannyFilter> CannyFilter::make(Info& info) {
	return std::shared_ptr<CannyFilter>(new CannyFilter{ info });
}

void CannyFilter::updateinfo(Info& info) {
	canny_filter->SetVariance(info.variance);
	canny_filter->SetLowerThreshold(info.lower_bound);
	canny_filter->SetUpperThreshold(info.upper_bound);
}

Internal2DImageType* CannyFilter::get_output() {
	return cast_to_char->GetOutput();
}

void CannyFilter::set_input(const Internal2DImageType* val_in) {
	cast_to_real->SetInput(val_in);
	canny_filter->SetInput(cast_to_real->GetOutput());
	cast_to_char->SetInput(canny_filter->GetOutput());
}

Internal2DImageType::Pointer CannyFilter::update_and_return_out() {
	try
	{
		cast_to_char->Update();
	}
	catch (const itk::ExceptionObject& err)
	{
		return nullptr;
	}
	return cast_to_char->GetOutput();
}

BinarizeFilter::BinarizeFilter(Info& info) {
	filter = FilterType::New();
	updateinfo(info);
}

std::shared_ptr<BinarizeFilter>BinarizeFilter::make(Info& info) {
	return std::shared_ptr<BinarizeFilter>(new BinarizeFilter{ info });
}

void BinarizeFilter::updateinfo(Info& info) {
	filter->SetOutsideValue(info.outside_value);
	filter->SetInsideValue(info.inside_value);
	filter->SetLowerThreshold(info.lower_value);
	filter->SetUpperThreshold(info.higher_value);
}

Internal2DImageType* BinarizeFilter::get_output() {
	return filter->GetOutput();
}

void BinarizeFilter::set_input(const Internal2DImageType* val_in) {
	filter->SetInput(val_in);
}

Internal2DImageType::Pointer BinarizeFilter::update_and_return_out() {
	try
	{
		filter->Update();
	}
	catch (const itk::ExceptionObject& err)
	{
		return nullptr;
	}
	return filter->GetOutput();
}

}
}
}