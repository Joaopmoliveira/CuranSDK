#include "MessageProcessing.h"

bool process_transform_message(ProcessingMessage *processor, igtl::MessageBase::Pointer val)
{
	igtl::TransformMessage::Pointer transform_message = igtl::TransformMessage::New();
	transform_message->Copy(val);
	int c = transform_message->Unpack(1);
	if (!(c & igtl::MessageHeader::UNPACK_BODY))
		return false; // failed to unpack message, therefore returning without doing anything
	return true;
}

bool process_image_message(ProcessingMessage *processor, igtl::MessageBase::Pointer val)
{
	return true;
}

bool process_tracking_message(ProcessingMessage *processor, igtl::MessageBase::Pointer val)
{
	igtl::TrackingDataMessage::Pointer trackingData;
	trackingData = igtl::TrackingDataMessage::New();
	trackingData->Copy(val);
	int c = trackingData->Unpack(1);

	if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
	{
		int nElements = trackingData->GetNumberOfTrackingDataElements();
		if(nElements<1)
			throw std::runtime_error("there are not enough tracking elements in the incoming message");
		igtl::TrackingDataElement::Pointer trackingElement;
		trackingData->GetTrackingDataElement(0, trackingElement);

		igtl::Matrix4x4 local_mat;
		trackingElement->GetMatrix(local_mat);
		ObservationEigenFormat observation_n;
		for (size_t cols = 0; cols < 4; ++cols)
			for (size_t lines = 0; lines < 4; ++lines)
				observation_n.flange_data(lines, cols) = local_mat[lines][cols];

		if(processor->should_record_point_for_calibration){
			processor->push_calibration_point(observation_n);
			processor->record_calib_button->set_waiting_color(SK_ColorBLACK);
			processor->should_record_point_for_calibration = false;
		}

		if(processor->should_record_point_from_world){
			processor->push_world_point(observation_n);
			processor->record_world_button->set_waiting_color(SK_ColorBLACK);
			processor->should_record_point_from_world = false;
		}

		return 1;
	}
	return 0;

	return true;
}

std::map<std::string, std::function<bool(ProcessingMessage *, igtl::MessageBase::Pointer val)>> openigtlink_callbacks{
	{"TRANSFORM", process_transform_message},
	{"IMAGE", process_image_message},
	{"TDATA",process_tracking_message}};

bool ProcessingMessage::process_message(size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
{
	// std::cout << "received message\n";
	assert(val.IsNotNull());
	if (er)
	{
		return true;
	}
	if (auto search = openigtlink_callbacks.find(val->GetMessageType()); search != openigtlink_callbacks.end())
		search->second(this, val);
	else
		std::cout << "No functionality for function received\n";
	return false;
}

void ProcessingMessage::communicate()
{
	std::cout << "connecting to server" << std::endl;
	clear_calibration_points();
	clear_world_point();
	connection_button->set_waiting_color(SK_ColorGREEN);
	io_context.restart();
	asio::ip::tcp::resolver resolver(io_context);
	client = curan::communication::Client<curan::communication::protocols::igtlink>::make(io_context, resolver.resolve("localhost", std::to_string(port)));
	connection_status.set(true);

	client->connect([this](size_t protocol_defined_val, std::error_code er, igtl::MessageBase::Pointer val)
					{
		try{
			if (process_message(protocol_defined_val, er, val))
			{
				attempt_stop();
			}
		}catch(...){
			std::cout << "Exception was thrown\n";
		} });

	igtl::StartTrackingDataMessage::Pointer startTrackingMsg = igtl::StartTrackingDataMessage::New();
	startTrackingMsg->SetDeviceName("ROBOT");
	startTrackingMsg->SetResolution(40);
	startTrackingMsg->SetCoordinateName("Base");
	startTrackingMsg->Pack();
	auto to_send = curan::utilities::CaptureBuffer::make_shared(startTrackingMsg->GetPackPointer(), startTrackingMsg->GetPackSize(), startTrackingMsg);
	client->write(to_send);

	io_context.run();
	connection_status.set(false);
	connection_button->set_waiting_color(SK_ColorRED);

	client.reset();
	return;
}

void ProcessingMessage::attempt_stop()
{
	igtl::StopTrackingDataMessage::Pointer stopTrackingMsg;
	stopTrackingMsg = igtl::StopTrackingDataMessage::New();
	stopTrackingMsg->SetDeviceName("ROBOT");
	stopTrackingMsg->Pack();

	auto to_send = curan::utilities::CaptureBuffer::make_shared(stopTrackingMsg->GetPackPointer(), stopTrackingMsg->GetPackSize(), stopTrackingMsg);
	if (client)
		client->write(to_send);
	io_context.stop();
}