#include "MessageProcessing.h"
#include "optimization/WireCalibration.h"

bool process_transform_message(ProcessingMessage* processor,igtl::MessageBase::Pointer val){
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; //failed to unpack message, therefore returning without doing anything
	return true;
}

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
	constexpr unsigned int Dimension = 2;
	using ImageType = itk::Image<PixelType, Dimension>;

	using FloatImageType = itk::Image<float, Dimension>;
	using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;
	auto importFilter = ImportFilterType::New();

	ImportFilterType::SizeType size;
	size[0] = x;
	size[1] = y;
	ImportFilterType::IndexType start;
	start.Fill(0);
	ImportFilterType::RegionType region;
	region.SetIndex(start);
	region.SetSize(size);

	importFilter->SetRegion(region);
	const itk::SpacePrecisionType origin[Dimension] = { 0.0, 0.0 };
	importFilter->SetOrigin(origin);
	const itk::SpacePrecisionType spacing[Dimension] = { 1.0, 1.0 };
	importFilter->SetSpacing(spacing);
	
	const bool importImageFilterWillOwnTheBuffer = false;
	importFilter->SetImportPointer((PixelType*)message_body->GetScalarPointer(), message_body->GetScalarSize(), importImageFilterWillOwnTheBuffer);

	try {
		importFilter->Update();
	}
	catch (...) {
		return false;
	}

	// where you have imported the raw image buffer into a ITK compatible image, thus you can use this for your purpouses

	ObservationEigenFormat observation_n;

	igtl::Matrix4x4 local_mat;
	message_body->GetMatrix(local_mat);
	for (size_t cols = 0; cols < 4; ++cols)
		for (size_t lines = 0; lines < 4; ++lines)
			observation_n.flange_data(lines, cols) = local_mat[lines][cols];

	ImageType::Pointer shr_ptr_imported = importFilter->GetOutput();
    ImageType::SizeType size_itk =  shr_ptr_imported->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(shr_ptr_imported->GetBufferPointer(),shr_ptr_imported->GetPixelContainer()->Size()*sizeof(char),shr_ptr_imported);
    curan::ui::ImageWrapper wrapper{buff,size_itk[0],size_itk[1]};
	static double timer = 0.0;
	processor->processed_viwer->update_batch([&](SkCanvas* canvas, SkRect image_area, SkRect widget_area){
		SkPaint paint;
    	paint.setStyle(SkPaint::kFill_Style);
    	paint.setAntiAlias(true);
    	paint.setStrokeWidth(30.0);
    	paint.setColor(SK_ColorRED);
		canvas->drawCircle(SkPoint::Make(image_area.centerX()+image_area.width()*0.3*(std::cos(timer)),
										 image_area.centerY()+image_area.height()*0.3*(std::sin(timer))),20.0,paint);
		const std::string text = "This is an example";
		const char* fontFamily = nullptr;  // Default system family, if it exists.
    	SkFontStyle fontStyle;  // Default is normal weight, normal width,  upright slant.
		sk_sp<SkFontMgr> fontManager = SkFontMgr::RefDefault();
    	sk_sp<SkTypeface> typeface = fontManager->legacyMakeTypeface(fontFamily, fontStyle);

    	SkFont font1(typeface, 64.0f, 1.0f, 0.0f);
    	font1.setEdging(SkFont::Edging::kAntiAlias);

		canvas->drawSimpleText(text.data(),text.size(),SkTextEncoding::kUTF8,image_area.fLeft,image_area.centerY(),font1,paint);
	},wrapper);

	timer += 0.1;

	end = std::chrono::steady_clock::now();
	auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	if(time_elapsed>40)
		std::printf("warning: reduce brightness of image because processing size is too large (%lld milliseconds)\n",time_elapsed);
	return true;
}

std::map<std::string,std::function<bool(ProcessingMessage*,igtl::MessageBase::Pointer)>> openigtlink_callbacks{
	{"TRANSFORM",process_transform_message},
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
	list_of_recorded_points.clear();
	using namespace curan::communication;
	button->set_waiting_color(SK_ColorGREEN);
	io_context.reset();
	interface_igtl igtlink_interface;
	Client::Info construction{ io_context,igtlink_interface };
	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
	construction.endpoints = endpoints;
	Client client{ construction };
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
	auto connectionstatus = client.connect(lam);
	io_context.run();
	button->set_waiting_color(SK_ColorRED);
	return;
}

void ProcessingMessage::attempt_stop() {
	io_context.stop();
}