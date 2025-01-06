#include "communication/Client.h"
#include "communication/ProtoIGTL.h"
#include "communication/Server.h"
#include "imageprocessing/IntegratedVolumeReconstructor.h"
#include "imageprocessing/igtl2itkConverter.h"
#include "itkImportImageFilter.h"
#include "rendering/Renderable.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "utils/Logger.h"
#include <asio.hpp>
#include <csignal>
#include <itkImage.h>
#include <map>
#include <nlohmann/json.hpp> // Include the JSON library
#include <optional>
#include <random> // Include the random header for generating random numbers

struct VolumeInformation {
  const long width = 400;
  const long height = 400;
  float spacing[3];
  float final_spacing[3];

  VolumeInformation(int in_width) : width{in_width}, height{in_width} {
    spacing[0] = 1.0 / width;
    spacing[1] = 1.0 / width;
    spacing[2] = 1.0 / width;

    final_spacing[0] = 2.0 / 300;
    final_spacing[1] = 2.0 / 300;
    final_spacing[2] = 2.0 / 300;
  }
};

using imagetype = itk::Image<unsigned char, 3>;
using IteratorType = itk::ImageRegionIteratorWithIndex<imagetype>;

auto allocate_image(double *euler_angles, const VolumeInformation &info) {
  imagetype::Pointer image = imagetype::New();
  itk::Matrix<double> image_orientation;
  itk::Point<double> image_origin;

  image_origin[0] = 0.0;
  image_origin[1] = 0.0;
  image_origin[2] = 0.0;

  imagetype::IndexType start;
  start[0] = 0; // first index on X
  start[1] = 0; // first index on Y
  start[2] = 0; // first index on Z

  imagetype::SizeType size;
  size[0] = info.width;  // size along X
  size[1] = info.height; // size along Y
  size[2] = 1;           // size along Z

  imagetype::RegionType region_1;
  region_1.SetSize(size);
  region_1.SetIndex(start);

  euler_angles[0] += 0.025;

  double t1 = cos(euler_angles[2]);
  double t2 = sin(euler_angles[2]);
  double t3 = cos(euler_angles[1]);
  double t4 = sin(euler_angles[1]);
  double t5 = cos(euler_angles[0]);
  double t6 = sin(euler_angles[0]);

  image_orientation(0, 0) = t1 * t3;
  image_orientation(1, 0) = t2 * t3;
  image_orientation(2, 0) = -t4;

  image_orientation(0, 1) = t1 * t4 * t6 - t2 * t5;
  image_orientation(1, 1) = t1 * t5 + t2 * t4 * t6;
  image_orientation(2, 1) = t3 * t6;

  image_orientation(0, 2) = t2 * t6 + t1 * t4 * t5;
  image_orientation(1, 2) = t2 * t4 * t5 - t1 * t6;
  image_orientation(2, 2) = t3 * t5;

  image->SetRegions(region_1);
  image->SetDirection(image_orientation);
  image->SetOrigin(image_origin);
  image->SetSpacing(info.spacing);
  image->Allocate();

  IteratorType outputIt(image, image->GetRequestedRegion());
  for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt) {
    imagetype::IndexType idx = outputIt.GetIndex();
    auto val = (double)idx[0] * idx[0] + (double)idx[1] * idx[1];
    val /=
        (double)info.height * info.height + (double)info.height * info.height;
    val *= 255.0;
    outputIt.Set(val);
  }
  return image;
}

void update_image_coordiantes(imagetype::Pointer &image,
                              double *previous_euler_angles,
                              const VolumeInformation &info) {
  itk::Matrix<double> image_orientation;
  itk::Point<double> image_origin;

  image_origin[0] = 0.0;
  image_origin[1] = 0.0;
  image_origin[2] = 0.0;

  imagetype::IndexType start;
  start[0] = 0; // first index on X
  start[1] = 0; // first index on Y
  start[2] = 0; // first index on Z

  imagetype::SizeType size;
  size[0] = info.width;  // size along X
  size[1] = info.height; // size along Y
  size[2] = 1;           // size along Z

  imagetype::RegionType region_1;
  region_1.SetSize(size);
  region_1.SetIndex(start);

  previous_euler_angles[0] += 0.05;

  double t1 = cos(previous_euler_angles[2]);
  double t2 = sin(previous_euler_angles[2]);
  double t3 = cos(previous_euler_angles[1]);
  double t4 = sin(previous_euler_angles[1]);
  double t5 = cos(previous_euler_angles[0]);
  double t6 = sin(previous_euler_angles[0]);

  image_orientation(0, 0) = t1 * t3;
  image_orientation(1, 0) = t2 * t3;
  image_orientation(2, 0) = -t4;

  image_orientation(0, 1) = t1 * t4 * t6 - t2 * t5;
  image_orientation(1, 1) = t1 * t5 + t2 * t4 * t6;
  image_orientation(2, 1) = t3 * t6;

  image_orientation(0, 2) = t2 * t6 + t1 * t4 * t5;
  image_orientation(1, 2) = t2 * t4 * t5 - t1 * t6;
  image_orientation(2, 2) = t3 * t5;

  image->SetRegions(region_1);
  image->SetDirection(image_orientation);
  image->SetOrigin(image_origin);
  image->SetSpacing(info.spacing);
}

asio::io_context io_context;

void signal_handler(int signal) { io_context.stop(); std::cout << "thrown exception:" << std::endl;  }

std::vector<double> timings;
 
void post_client_message(const std::string &message,std::shared_ptr<
        curan::communication::Client<curan::communication::protocols::igtlink>>
        &client) {
  static size_t identifier = 0;
  if(timings.size()>102)
    std::raise(SIGINT);

  igtl::StringMessage::Pointer command = igtl::StringMessage::New();
  command->SetDeviceName("CMD_" + std::to_string(identifier));
  command->SetString(message);
  command->Pack();
  auto to_send_string = curan::utilities::CaptureBuffer::make_shared(
      command->GetPackPointer(), command->GetPackSize(), command);
  client->write(to_send_string);
  std::cout << "sent command to start reconstruction: " << command->GetDeviceName() << std::endl;
  ++identifier;
}

void post_client(
    std::shared_ptr<
        curan::communication::Client<curan::communication::protocols::igtlink>>
        &client,
    asio::ip::tcp::resolver &fri_resolver,
    std::map<std::string, std::function<bool(igtl::MessageBase::Pointer val)>>
        &openigtlink_callbacks) {
  client = curan::communication::
      Client<curan::communication::protocols::igtlink>::make(
          io_context, fri_resolver.resolve("localhost", std::to_string(18944)),
          std::chrono::seconds(10), [&](std::error_code ec) {
            if (ec) {
              std::cout << "failure detected:\n" << ec.message();
              io_context.post([&]() {
                post_client(client, fri_resolver, openigtlink_callbacks);
              });
              return;
            }
            std::cout << "no failure detected:\n";

            if (client)
              client->connect([&](size_t protocol_defined_val,
                                  std::error_code er,
                                  igtl::MessageBase::Pointer val) {
                if (er) {
                  return true;
                }
                if (auto search =
                        openigtlink_callbacks.find(val->GetMessageType());
                    search != openigtlink_callbacks.end())
                  search->second(val);
                else
                  std::cout << "No functionality for function received: "
                            << val->GetDeviceType() << "\n";
                return false;
              });

            post_client_message(
                "<Command Name=\"StartVolumeReconstruction\" "
                "VolumeReconstructorDeviceId=\"VolumeReconstructorDevice\" />",
                client);
          });
}

int main() {
   timings.reserve(100);
  auto get_current_value = []() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
               .count() * 1e-6;
  };

  std::signal(SIGINT, signal_handler);
  double euler_angles[3] = {0, 0, 0};
  VolumeInformation volume_size_info{1000};
  auto image = allocate_image(&euler_angles[0], volume_size_info);
  std::atomic<size_t> identifier = 0;
  std::shared_ptr<
      curan::communication::Client<curan::communication::protocols::igtlink>>
      client = nullptr;
  curan::renderable::Window::Info info;
  info.api_dump = false;
  info.display = "";
  info.full_screen = false;
  info.is_debug = false;
  info.screen_number = 0;
  info.title = "myviewer";
  curan::renderable::Window::WindowSize size{1000, 800};
  info.window_size = size;
  curan::renderable::Window window{info};

  std::array<double, 3> vol_origin = {0.05, 0.05, 0.05};
  std::array<double, 3> vol_spacing = {0.007000, 0.007000, 0.007000};
  std::array<double, 3> vol_size = {300 * 0.007000, 300 * 0.007000,
                                    300 * 0.007000};
  std::array<std::array<double, 3>, 3> vol_direction;
  vol_direction[0] = {1.0, 0.0, 0.0};
  vol_direction[1] = {0.0, 1.0, 0.0};
  vol_direction[2] = {0.0, 0.0, 1.0};

  curan::image::IntegratedReconstructor::Info recon_info{
      vol_spacing, vol_origin, vol_size, vol_direction};
  vsg::ref_ptr<curan::renderable::Renderable> integrated_volume =
      curan::image::IntegratedReconstructor::make(recon_info);
  integrated_volume->cast<curan::image::IntegratedReconstructor>()
      ->set_compound(
          curan::image::reconstruction::Compounding::LATEST_COMPOUNDING_MODE)
      .set_interpolation(
          curan::image::reconstruction::Interpolation::LINEAR_INTERPOLATION);
  window << integrated_volume;
  // message sent / timestamp of sending / timestamp of receiving
  std::mutex mut;
  std::tuple<bool, double, bool, double> f_atomic_values =
      std::make_tuple(false, 0.0, false, 0.0);


  asio::ip::tcp::resolver fri_resolver(io_context);

  std::map<std::string, std::function<bool(igtl::MessageBase::Pointer val)>>
      openigtlink_callbacks{
          {"TRANSFORM", [](igtl::MessageBase::Pointer val) { return true; }},
          {"STATUS",
           [](igtl::MessageBase::Pointer val) {
             igtl::StatusMessage::Pointer status_message =
                 igtl::StatusMessage::New();
             status_message->Copy(val);
             int c = status_message->Unpack(1);
             if (!(c & igtl::MessageHeader::UNPACK_BODY))
               return false;
             if (status_message->GetCode() != 1)
               std::cout << "status: " << status_message->GetErrorName()
                         << std::endl;
             return true;
           }},
          {"STRING",
           [&](igtl::MessageBase::Pointer val) {
             igtl::StringMessage::Pointer string_message =
                 igtl::StringMessage::New();
             string_message->Copy(val);
             int c = string_message->Unpack(1);
             if (!(c & igtl::MessageHeader::UNPACK_BODY))
               return false;
             std::cout << "string message: " << string_message->GetString()
                       << std::endl;
             return true;
           }},
          {"IMAGE", [&](igtl::MessageBase::Pointer val) {
             igtl::ImageMessage::Pointer image_message =
                 igtl::ImageMessage::New();
             image_message->Copy(val);
             int c = image_message->Unpack(1);
             if (!(c & igtl::MessageHeader::UNPACK_BODY))
               return false;
             int size[3];
             image_message->GetDimensions(size);

             if (size[2] < 10)
               return true;

             float s[3];
             image_message->GetSpacing(s);
             float p[3];
             image_message->GetOrigin(p);

             std::lock_guard<std::mutex> g{mut};
             size_t summation = 0;
             integrated_volume->cast<curan::image::IntegratedReconstructor>()
                 ->update_volume([&](vsg::floatArray3D &image) {
                   auto raw_pixel = image_message->GetScalarPointer();
                   for (size_t d = 0; d < image.depth(); ++d)
                     for (size_t r = 0; r < image.height(); ++r)
                       for (size_t c = 0; c < image.width(); ++c) {
                         float v = *(d * size[0] * size[1] + r * size[0] + c +
                                     (unsigned char *)raw_pixel);
                         summation += v;
                         image.set(c, r, d, v);
                       }
                 });

             static size_t hash_summation = summation;

             if (hash_summation == summation) {
                std::printf("data timing due to samies: %llu \n",summation);
                return true;
             } 
             hash_summation = summation;
             auto [was_image_sent, timestamp_proper, was_string_sent, to_set] = f_atomic_values;

             std::printf("data timing: %f %llu \n",get_current_value() - timestamp_proper, summation);
             f_atomic_values = std::make_tuple(false, 0.0, false, 0.0);
             timings.push_back(get_current_value() - timestamp_proper);

             update_image_coordiantes(image, &euler_angles[0],
                                      volume_size_info);

             return true;
           }}};
  std::tuple<bool, bool> lauched_code = std::make_tuple(false, false);

  auto image_server =
      curan::communication::Server<curan::communication::protocols::igtlink>::
          make(io_context, 50000, [&](std::error_code ec) {
            if (ec) {
              std::cout << "image server failure";
              return true;
            }
            std::cout << "image server received!";
            auto [img_server, trnsf_server] = lauched_code;
            if (trnsf_server) // should lauch client
            {
              io_context.post([&]() {
                post_client(client, fri_resolver, openigtlink_callbacks);
              });
            }
            lauched_code = std::make_tuple(true, trnsf_server);
            return true;
          });
  auto transform_server =
      curan::communication::Server<curan::communication::protocols::igtlink>::
          make(io_context, 50010, [&](std::error_code ec) {
            if (ec) {
              std::cout << "transform server failure";
              return true;
            }
            std::cout << "transform server received!";
            auto [img_server, trnsf_server] = lauched_code;
            if (img_server) // should lauch client
            {
              io_context.post([&]() {
                post_client(client, fri_resolver, openigtlink_callbacks);
              });
            }
            lauched_code = std::make_tuple(img_server, true);
            return true;
          });

  auto pool = curan::utilities::ThreadPool::create(2);
  pool->submit(curan::utilities::Job{"running context", [&]() {
                                       auto guard =
                                           asio::make_work_guard(io_context);
                                       io_context.run();
                                     }});

  std::atomic<size_t> previous_summation = 0;

  pool->submit(curan::utilities::Job{
      "running message creator", [&]() {
        igtl::TimeStamp::Pointer ts = igtl::TimeStamp::New();

        int size[] = {volume_size_info.width, volume_size_info.height, 1};
        float spacing[] = {volume_size_info.spacing[0],
                           volume_size_info.spacing[1],
                           volume_size_info.spacing[2]};
        int svsize[] = {volume_size_info.width, volume_size_info.height, 1};
        int svoffset[] = {0, 0, 0};
        int scalarType = igtl::ImageMessage::TYPE_UINT8;
        size_t counter = 0;
        while (!io_context.stopped()) {
          ts->GetTime();

          igtl::ImageMessage::Pointer imgMsg = igtl::ImageMessage::New();
          imgMsg->SetDimensions(size);
          imgMsg->SetSpacing(spacing);
          imgMsg->SetScalarType(scalarType);
          imgMsg->SetDeviceName("VideoDevice");
          imgMsg->SetSubVolume(svsize, svoffset);
          imgMsg->AllocateScalars();

          std::memcpy(imgMsg->GetScalarPointer(), image->GetBufferPointer(),
                      image->GetLargestPossibleRegion().GetSize()[0] *
                          image->GetLargestPossibleRegion().GetSize()[1] *
                          image->GetLargestPossibleRegion().GetSize()[2]);

          igtl::Matrix4x4 image_transform;
          image_transform[3][0] = 0.0;
          image_transform[3][1] = 0.0;
          image_transform[3][2] = 0.0;
          image_transform[3][3] = 1.0;

          for (size_t row = 0; row < 3; ++row)
            for (size_t col = 0; col < 3; ++col)
              image_transform[row][col] = image->GetDirection()(row, col);

          image_transform[0][3] = image->GetOrigin()[0];
          image_transform[1][3] = image->GetOrigin()[1];
          image_transform[2][3] = image->GetOrigin()[2];

          imgMsg->SetMatrix(image_transform);
          imgMsg->Pack();

          igtl::TransformMessage::Pointer transMsg =
              igtl::TransformMessage::New();
          transMsg->SetDeviceName("FlangeToReference");

          transMsg->SetMatrix(image_transform);
          transMsg->SetTimeStamp(ts);
          transMsg->Pack();

          auto to_send_transf = curan::utilities::CaptureBuffer::make_shared(
              transMsg->GetPackPointer(), transMsg->GetPackSize(), transMsg);
          auto to_send_img = curan::utilities::CaptureBuffer::make_shared(
              imgMsg->GetPackPointer(), imgMsg->GetPackSize(), imgMsg);

          image_server->write(to_send_img);
          transform_server->write(to_send_transf);
          {
            std::lock_guard<std::mutex> g{mut};
            auto [was_image_sent, timestamp_proper, was_string_sent, to_set] =
                f_atomic_values;
            if (counter > 5) {
              post_client_message(
                  "<Command Name=\"GetVolumeReconstructionSnapshot\" "
                  "VolumeReconstructorDeviceId=\"VolumeReconstructorDevice\"  "
                  "ApplyHoleFilling=\"FALSE\" "
                  "OutputVolDeviceName=\"recvol_Reference_temp\"/>",
                  client);
            }
            if (!was_image_sent) {
              f_atomic_values =
                  std::make_tuple(true, get_current_value(), false, 0.0);
            } else {
              std::cout << "not seding message\n";
            }
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(510));

          ++counter;
        }
      }});

  for (; !io_context.stopped(); window.run_once()) {
  }

  {
    std::lock_guard<std::mutex> g{mut};
    for(const auto val : timings){
        std::printf("%f , ",val);
    }
  }


  post_client_message(
      "<Command Name=\"StopVolumeReconstruction\" "
      "VolumeReconstructorDeviceId=\"VolumeReconstructorDevice\" "
      "OutputVolDeviceName=\"recvol_Reference\" "
      "OutputVolFilename=\"recvol_Reference.mha\"/>",
      client);


  std::raise(SIGINT);
  return 0;
}