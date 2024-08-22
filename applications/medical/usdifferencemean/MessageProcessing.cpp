#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"

constexpr double scaling_factor = 1e4;

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
					accumulator += (1.0/scaling_factor)*val.Get();
				double mean = (scaling_factor)*(accumulator/averaged_images_iterators.size());
        		outputIterator.Set(mean);

        		for(auto& val : averaged_images_iterators)
					++val;
				++outputIterator;
    		}

			processor->mean_image_computed = mean_image_computation;
    		auto buff = curan::utilities::CaptureBuffer::make_shared(importFilter->GetOutput()->GetBufferPointer(),importFilter->GetOutput()->GetPixelContainer()->Size()*sizeof(char),importFilter->GetOutput());
    		curan::ui::ImageWrapper wrapper{buff,importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0],importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};
			processor->difference_image->update_image(wrapper);
			processor->accepted_region_image->update_image(wrapper);
			processor->images_recorded = std::list<ImageType::Pointer>{};

		} else{
    		auto buff = curan::utilities::CaptureBuffer::make_shared(importFilter->GetOutput()->GetBufferPointer(),importFilter->GetOutput()->GetPixelContainer()->Size()*sizeof(char),importFilter->GetOutput());
    		curan::ui::ImageWrapper wrapper{buff,importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0],importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};
			processor->difference_image->update_image(wrapper);
			processor->accepted_region_image->update_image(wrapper);
		}
	}
	else if(!processor->average_inactive_differences){
		importFilter->Update();
		processor->images_recorded.push_back(importFilter->GetOutput());

		ProcessingMessage::ImageType::Pointer deep_copy_for_averaging = ProcessingMessage::ImageType::New();
		deep_copy_for_averaging->SetRegions(importFilter->GetOutput()->GetLargestPossibleRegion());
    	deep_copy_for_averaging->Allocate();
    	deep_copy_for_averaging->SetSpacing(importFilter->GetOutput()->GetSpacing());
    	deep_copy_for_averaging->SetOrigin(importFilter->GetOutput()->GetOrigin());
    	deep_copy_for_averaging->SetDirection(importFilter->GetOutput()->GetDirection());

    	itk::ImageRegionConstIterator<ProcessingMessage::ImageType> inputIterator(importFilter->GetOutput(), importFilter->GetOutput()->GetLargestPossibleRegion());
		itk::ImageRegionConstIterator<ProcessingMessage::ImageType> meanIterator(*processor->mean_image_computed, (*processor->mean_image_computed)->GetLargestPossibleRegion());
		itk::ImageRegionIterator<ProcessingMessage::ImageType> outputIteratorAveraging(deep_copy_for_averaging, deep_copy_for_averaging->GetLargestPossibleRegion());
		
    	while (!inputIterator.IsAtEnd())
    	{
			outputIteratorAveraging.Set(std::abs(inputIterator.Get()-meanIterator.Get()));
        	++inputIterator;
			++meanIterator;
			++outputIteratorAveraging;
    	}
		Eigen::Map<Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> mapped_matrix{deep_copy_for_averaging->GetBufferPointer(),(int)deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[1],(int)deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[0]};


		constexpr size_t n_blocks = 10;
		std::vector<std::tuple<size_t,double>> average_differences;
		std::vector<double> regions;
		auto mapped_matrix_converted = mapped_matrix.cast<double>()*1/(scaling_factor);
		size_t x_step = std::ceil(mapped_matrix_converted.cols()/(double)n_blocks);
		size_t x_index = 0;
		for(size_t i = 0; x_index<mapped_matrix_converted.cols(); ++i, x_index+=x_step ){
			int x_boundary = (x_index+x_step > mapped_matrix_converted.cols()) ? mapped_matrix_converted.cols()-x_index : x_step;
			if(x_boundary<=0)
				break;
			average_differences.emplace_back(x_boundary,scaling_factor*(mapped_matrix_converted.block(0,
																	x_index,	
																	mapped_matrix_converted.rows(),
																	x_boundary)
																	.sum() /(double)(x_boundary*mapped_matrix_converted.rows())));
			regions.emplace_back(scaling_factor*(mapped_matrix_converted.block(0,
																	x_index,	
																	mapped_matrix_converted.rows(),
																	x_boundary)
																	.sum() /(double)(x_boundary*mapped_matrix_converted.rows())));
		}

		processor->regions_recorded.push_back(regions);

		if(processor->regions_recorded.size()>10){
			std::vector<double> average_regions_difference;
			average_regions_difference.resize(regions.size());
			std::vector<double> standard_deviation_regions_difference;
			standard_deviation_regions_difference.resize(regions.size());
			std::printf("\nmean: ");
			for(size_t j = 0; j < regions.size(); ++j){
				for(const auto& previous_recording : processor->regions_recorded)
					average_regions_difference[j] += previous_recording[j];
				average_regions_difference[j] /=  processor->regions_recorded.size();
				std::printf(" %f ",average_regions_difference[j]);
			}
			std::printf("\ndeviation: ");
			for(size_t j = 0; j < regions.size(); ++j){
				for(const auto& previous_recording : processor->regions_recorded)
					standard_deviation_regions_difference[j] += std::pow(previous_recording[j]-average_regions_difference[j],2);
				standard_deviation_regions_difference[j] = std::sqrt((1.0/processor->regions_recorded.size())*standard_deviation_regions_difference[j]);
				average_regions_difference[j] += 1; // for now this standard deviation talk is not working, I will leave this threadhold here but this is not robust
				std::printf(" %f ",standard_deviation_regions_difference[j]);
			}
			std::printf("\n");
			processor->average_inactive_differences = average_regions_difference;
			processor->regions_recorded = std::list<std::vector<double>>{};
			processor->config_draw->stack_page->clear_overlays();
		}

		auto buff = curan::utilities::CaptureBuffer::make_shared(importFilter->GetOutput()->GetBufferPointer(),importFilter->GetOutput()->GetPixelContainer()->Size()*sizeof(char),importFilter->GetOutput());
    	curan::ui::ImageWrapper wrapper{buff,importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0],importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};
		processor->difference_image->update_image(wrapper);
		processor->accepted_region_image->update_image(wrapper);

	}
	else { // the mean image is already computed, therefore we just need to compute the difference image
		importFilter->Update();

		ProcessingMessage::ImageType::Pointer deep_copy_for_averaging = ProcessingMessage::ImageType::New();
		deep_copy_for_averaging->SetRegions(importFilter->GetOutput()->GetLargestPossibleRegion());
    	deep_copy_for_averaging->Allocate();
    	deep_copy_for_averaging->SetSpacing(importFilter->GetOutput()->GetSpacing());
    	deep_copy_for_averaging->SetOrigin(importFilter->GetOutput()->GetOrigin());
    	deep_copy_for_averaging->SetDirection(importFilter->GetOutput()->GetDirection());

    	itk::ImageRegionConstIterator<ProcessingMessage::ImageType> inputIterator(importFilter->GetOutput(), importFilter->GetOutput()->GetLargestPossibleRegion());
		itk::ImageRegionConstIterator<ProcessingMessage::ImageType> meanIterator(*processor->mean_image_computed, (*processor->mean_image_computed)->GetLargestPossibleRegion());
		itk::ImageRegionIterator<ProcessingMessage::ImageType> outputIteratorAveraging(deep_copy_for_averaging, deep_copy_for_averaging->GetLargestPossibleRegion());
		
    	while (!inputIterator.IsAtEnd())
    	{
			outputIteratorAveraging.Set(std::abs(inputIterator.Get()-meanIterator.Get()));
        	++inputIterator;
			++meanIterator;
			++outputIteratorAveraging;
    	}
		Eigen::Map<Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> mapped_matrix{deep_copy_for_averaging->GetBufferPointer(),(int)deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[1],(int)deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[0]};


		constexpr size_t n_blocks = 10;
		std::vector<std::tuple<size_t,double>> average_differences;
		auto mapped_matrix_converted = mapped_matrix.cast<double>()*1/(scaling_factor);
		size_t x_step = std::ceil(mapped_matrix_converted.cols()/(double)n_blocks);
		size_t x_index = 0;
		for(size_t i = 0; x_index<mapped_matrix_converted.cols(); ++i, x_index+=x_step ){
			int x_boundary = (x_index+x_step > mapped_matrix_converted.cols()) ? mapped_matrix_converted.cols()-x_index : x_step;
			if(x_boundary<=0)
				break;
			average_differences.emplace_back(x_boundary,scaling_factor*(mapped_matrix_converted.block(0,
																	x_index,	
																	mapped_matrix_converted.rows(),
																	x_boundary)
																	.sum() /(double)(x_boundary*mapped_matrix_converted.rows())));
		}

		auto buff = curan::utilities::CaptureBuffer::make_shared(deep_copy_for_averaging->GetBufferPointer(),deep_copy_for_averaging->GetPixelContainer()->Size()*sizeof(char),deep_copy_for_averaging);
    	curan::ui::ImageWrapper wrapper{buff,deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[0],deep_copy_for_averaging->GetLargestPossibleRegion().GetSize()[1]};
		size_t max_size_x = mapped_matrix.cols();

		auto buffraw = curan::utilities::CaptureBuffer::make_shared(importFilter->GetOutput()->GetBufferPointer(),importFilter->GetOutput()->GetPixelContainer()->Size()*sizeof(char),importFilter->GetOutput());
    	curan::ui::ImageWrapper wrapperraw{buffraw,importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0],importFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[1]};

		processor->accepted_region_image->update_batch([max_size_x,average_differences,&processor](SkCanvas* canvas, SkRect image_area,SkRect widget_area){
			float scalling_factor_x = image_area.width()/max_size_x;
            SkPaint linePaint;
            linePaint.setStyle(SkPaint::kFill_Style);
            linePaint.setAntiAlias(true);
            linePaint.setStrokeWidth(2.0f);  
            linePaint.setColor(SkColorSetARGB(80, 0x00, 0x00, 0xFF)); 
			size_t initial_value = image_area.fLeft;
			size_t index = 0;
			for(const auto& values:  average_differences){
				auto offset = scalling_factor_x*std::get<0>(values);
				if(std::get<1>(values)>(*processor->average_inactive_differences)[index]){
					SkRect belong_rect = SkRect::MakeXYWH(initial_value,image_area.fTop,offset,image_area.height());
					canvas->drawRect(belong_rect,linePaint);
				}
				++index;
				initial_value += offset;
			}
		},wrapperraw);

		processor->difference_image->update_batch([average_differences,max_size_x,&processor](SkCanvas* canvas, SkRect image_area,SkRect widget_area){
			float scalling_factor_x = image_area.width()/max_size_x;
            SkPaint linePaint;
            linePaint.setStyle(SkPaint::kFill_Style);
            linePaint.setAntiAlias(true);
            linePaint.setStrokeWidth(2.0f);  
            linePaint.setColor(SK_ColorBLUE); 
			size_t initial_value = image_area.fLeft;
			for(const auto& values:  average_differences){
				auto offset = scalling_factor_x*std::get<0>(values);
				SkRect original_rect = SkRect::MakeXYWH(initial_value,image_area.fTop,offset,image_area.height()*(std::get<1>(values)/255.0));
				canvas->drawRect(original_rect,linePaint);
				initial_value += offset;
			}
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
	asio::ip::tcp::resolver resolver(io_context);
	auto client = Client<curan::communication::protocols::igtlink>::make(io_context,resolver.resolve("localhost", std::to_string(port)));
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