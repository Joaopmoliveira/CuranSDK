#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"

bool process_image_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = begin;
	igtl::ImageMessage::Pointer message_body = igtl::ImageMessage::New();
	message_body->Copy(val);
	int c = message_body->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything

	int x, y, z;
	message_body->GetDimensions(x, y, z);
	using PixelType = unsigned char;
	constexpr unsigned int Dimension = 3;
	using ImageType = itk::Image<PixelType, Dimension>;

	using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;
	
	auto importFilter = ImportFilterType::New();

	ImportFilterType::SizeType size;
	size[0] = x;
	size[1] = y;
	size[2] = 1;
	ImportFilterType::IndexType start;
	start.Fill(0);
	ImportFilterType::RegionType region;
	region.SetIndex(start);
	region.SetSize(size);

	importFilter->SetRegion(region);
	const itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0 , 0.0 };
	importFilter->SetOrigin(origin);
	const itk::SpacePrecisionType spacing[Dimension] = { 1.0, 1.0 ,1.0 };
	importFilter->SetSpacing(spacing);
	
	const bool importImageFilterWillOwnTheBuffer = false;
	importFilter->SetImportPointer((PixelType*)message_body->GetScalarPointer(), message_body->GetScalarSize(), importImageFilterWillOwnTheBuffer);

	if (!processor->mean_image_computed ) {
		importFilter->Update();
		processor->images_recorded.push_back(importFilter->GetOutput());
		if(processor->images_recorded.size()>=10){
			std::vector<itk::ImageRegionConstIterator<ProcessingMessage::ImageType>> averaged_images_iterators;
			for(const auto& val : processor->images_recorded)
				averaged_images_iterators.emplace_back(val, val->GetLargestPossibleRegion());
			ProcessingMessage::ImageType::Pointer mean_image_computation = ProcessingMessage::ImageType::New();
			mean_image_computation->SetRegions(importFilter->GetOutput()->GetLargestPossibleRegion());
    		mean_image_computation->Allocate();
    		mean_image_computation->SetSpacing(importFilter->GetOutput()->GetSpacing());
    		mean_image_computation->SetOrigin(importFilter->GetOutput()->GetOrigin());
    		mean_image_computation->SetDirection(importFilter->GetOutput()->GetDirection());
    		itk::ImageRegionIterator<ImageType> outputIterator(mean_image_computation, mean_image_computation->GetLargestPossibleRegion());

    		while (!outputIterator.IsAtEnd()){
				double accumulator = 0.0;
				for(const auto& val : averaged_images_iterators)
					accumulator += val.Get();
        		outputIterator.Set(accumulator/averaged_images_iterators.size());
        		for(auto& val : averaged_images_iterators)
					++val;
				++outputIterator;
    		}

			using DiscreteFilterType = itk::MeanImageFilter<ImageType, ImageType>;
			DiscreteFilterType::Pointer discrete_filter_reverse = DiscreteFilterType::New();
			DiscreteFilterType::InputSizeType radius;
  			radius.Fill(2);

  			discrete_filter_reverse->SetRadius(radius);
  			discrete_filter_reverse->SetInput(mean_image_computation);
			
			discrete_filter_reverse->Update();
			processor->mean_image_computed = discrete_filter_reverse->GetOutput();

    		auto buff = curan::utilities::CaptureBuffer::make_shared(discrete_filter_reverse->GetOutput()->GetBufferPointer(),discrete_filter_reverse->GetOutput()->GetPixelContainer()->Size()*sizeof(char),discrete_filter_reverse->GetOutput());
    		curan::ui::ImageWrapper wrapper{buff,discrete_filter_reverse->GetOutput()->GetLargestPossibleRegion().GetSize()[0],discrete_filter_reverse->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};
			processor->difference_image->update_image(wrapper);

		} else{
    		auto buff = curan::utilities::CaptureBuffer::make_shared(importFilter->GetOutput()->GetBufferPointer(),importFilter->GetOutput()->GetPixelContainer()->Size()*sizeof(char),importFilter->GetOutput());
    		curan::ui::ImageWrapper wrapper{buff,importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0],importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};
			processor->difference_image->update_image(wrapper);
		}

	}
	else { // the mean image is already computed, therefore we just need to compute the difference image
		importFilter->Update();

		using DiscreteFilterType = itk::MeanImageFilter<ProcessingMessage::ImageType, ImageType>;
		DiscreteFilterType::Pointer discrete_filter_reverse = DiscreteFilterType::New();
		DiscreteFilterType::InputSizeType radius;
  		radius.Fill(2);

  		discrete_filter_reverse->SetRadius(radius);
  		discrete_filter_reverse->SetInput(importFilter->GetOutput());
		discrete_filter_reverse->Update();

		ProcessingMessage::ImageType::Pointer deep_copy_for_averaging = ProcessingMessage::ImageType::New();
		deep_copy_for_averaging->SetRegions(importFilter->GetOutput()->GetLargestPossibleRegion());
    	deep_copy_for_averaging->Allocate();
    	deep_copy_for_averaging->SetSpacing(importFilter->GetOutput()->GetSpacing());
    	deep_copy_for_averaging->SetOrigin(importFilter->GetOutput()->GetOrigin());
    	deep_copy_for_averaging->SetDirection(importFilter->GetOutput()->GetDirection());

    	itk::ImageRegionConstIterator<ProcessingMessage::ImageType> inputIterator(discrete_filter_reverse->GetOutput(), discrete_filter_reverse->GetOutput()->GetLargestPossibleRegion());
		itk::ImageRegionConstIterator<ProcessingMessage::ImageType> meanIterator(*processor->mean_image_computed, (*processor->mean_image_computed)->GetLargestPossibleRegion());
		itk::ImageRegionIterator<ProcessingMessage::ImageType> outputIteratorAveraging(deep_copy_for_averaging, deep_copy_for_averaging->GetLargestPossibleRegion());
		
    	while (!inputIterator.IsAtEnd())
    	{
			outputIteratorAveraging.Set(std::abs(inputIterator.Get()-meanIterator.Get()));
        	++inputIterator;
			++meanIterator;
			++outputIteratorAveraging;
    	}

		auto buff = curan::utilities::CaptureBuffer::make_shared(deep_copy_for_averaging->GetBufferPointer(),deep_copy_for_averaging->GetPixelContainer()->Size()*sizeof(char),deep_copy_for_averaging);
    	curan::ui::ImageWrapper wrapper{buff,deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[0],deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[1]};
		double size_cols = deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[1];

		processor->difference_image->update_batch([](SkCanvas* canvas, SkRect image_area,SkRect widget_area){ 
            SkPaint linePaint;
            linePaint.setStyle(SkPaint::kStroke_Style);
            linePaint.setAntiAlias(false);
            linePaint.setStrokeWidth(1.0f);  
			linePaint.setColor(SK_ColorRED); 
			/*
			static bool is_initializing = true;
			static SkPath path;
			static std::vector<SkPoint> list_of_points;
			static size_t previous_size = cols_accumulation.cols();
		
			if(is_initializing){
				list_of_points.resize(cols_accumulation.cols());
				path.setIsVolatile(true);
				path.addPoly(list_of_points.data(),list_of_points.size(),false);
				is_initializing = false;
			}

			if(previous_size != cols_accumulation.cols()){
				list_of_points.resize(cols_accumulation.cols());
				path.setIsVolatile(true);
				path.addPoly(list_of_points.data(),list_of_points.size(),false);
			}

			double x_pos = image_area.fLeft;
			size_t index = 0;
			for(auto& val : list_of_points){
				val.fX = x_pos;
				val.fY = image_area.fTop+image_area.height()*(cols_accumulation[index]/maximum_value);
				x_pos += scalling_factor_x;
				++index;
			}
			canvas->drawPath(path, linePaint);
			*/

    	},wrapper);

	}
	end = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	if(time_elapsed>40)
		std::printf("warning: reduce brightness of image because processing size is too large (%lld milliseconds)\n",time_elapsed);
	return true;
}

std::map<std::string,std::function<bool(ProcessingMessage*,igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"IMAGE",process_image_message}
};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
	assert(val.IsNotNull());
	if (er){
        return true;
    } 
    if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
        search->second(this,val);
    else
        std::cout << "No functionality for function received\n";
    return false;
}

void ProcessingMessage::communicate() {
	images_recorded.clear();
	using namespace curan::communication;
	button->set_waiting_color(SK_ColorGREEN);
	io_context.reset();
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	auto client = Client::make(construction);
	connection_status.set(true);

	auto lam = [this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val) {
		try{
			if (process_message(protocol_defined_val, er, val))
			{
				connection_status.set(false);
				attempt_stop();
			}
		}catch(...){
			std::cout << "Exception was thrown\n";
		}
	};
	client->connect(lam);
	io_context.run();
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}