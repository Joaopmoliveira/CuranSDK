#include "imageprocessing/FilterAlgorithms.h"

namespace curan {
namespace image {
namespace filtering {

Filter::Filter() {
	input = Internal2DImageType::New();
}

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

bool Filter::set_input(Internal2DImageType::Pointer supplied_input) {
	if (filters.size() == 0) {
		return false;
	}
	auto val = filters.begin();
	(*val)->set_input(supplied_input);
	if (filters.size() == 1)
		return true;

	auto previous = val; 
	++val;
	for (; val!=filters.end(); ++val, ++previous)
		(*val)->set_input((*previous)->get_output());
	return true;
}

Internal2DImageType::Pointer Filter::get_output(){
	if (filters.size() == 0)
		return nullptr;
	auto temp = filters.back();
	return temp->update_and_return_out();

};

ImportFilter::ImportFilter(Info& info) {
	filter = ImportFilterType::New();
	updateinfo(info);
};

std::shared_ptr<ImportFilter> ImportFilter::make(Info& info) {
	return std::shared_ptr<ImportFilter>(new ImportFilter{ info });
}

void ImportFilter::updateinfo(Info& info) {
	ImportFilterType::RegionType region;
	region.SetIndex(info.start);
	region.SetSize(info.size);
	filter->SetRegion(region);
	itk::SpacePrecisionType origin[2];
	origin[0] = info.origin[0];
	origin[1] = info.origin[1];
	filter->SetOrigin(origin);
	itk::SpacePrecisionType spacing[2];
	spacing[0] = info.spacing[0];
	spacing[1] = info.spacing[1];
	filter->SetSpacing(spacing);
	filter->SetImportPointer(info.buffer, info.number_of_pixels, info.memory_owner);
}

Internal2DImageType* ImportFilter::get_output() {
	return filter->GetOutput();
}

void ImportFilter::set_input(const Internal2DImageType* in_val) {

}

Internal2DImageType::Pointer ImportFilter::update_and_return_out() {
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


CircleFilter::CircleFilter(Info& info) {

}

std::shared_ptr<CircleFilter> CircleFilter::make(Info& info) {

}

void CircleFilter::updateinfo(Info& info) {

}

Internal2DImageType* CircleFilter::get_output() {


}

void CircleFilter::set_input(const Internal2DImageType*){

}

Internal2DImageType::Pointer CircleFilter::update_and_return_out()
{

}

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