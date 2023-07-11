#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include <thread>
#include <csignal>
#include <chrono>
#include "utils/Logger.h"
#include <atomic>
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <iostream>
#include <optional>
#include <map>
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkImage.h"
#include "itkImportImageFilter.h"
#include <atomic>
#include "imageprocessing/VolumeReconstructor.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using ImportFilterType = itk::ImportImageFilter<PixelType, Dimension>;

using OutputPixelType = float;
using InputImageType = itk::Image<unsigned char, 3>;
using OutputImageType = itk::Image<OutputPixelType, 3>;
using FilterType = itk::CastImageFilter<InputImageType, OutputImageType>;

struct SharedState{
    curan::image::StaticReconstructor reconstructor;
    std::optional<vsg::ref_ptr<curan::renderable::Renderable>> texture;
    vsg::ref_ptr<curan::renderable::Renderable> volume;
    curan::renderable::Window & window;
    asio::io_context& context;
    SharedState(curan::renderable::Window & in_window, asio::io_context& in_context,const curan::image::StaticReconstructor::Info& info, vsg::ref_ptr<curan::renderable::Renderable> in_volume) : window{in_window},context{in_context},reconstructor{info}, volume{in_volume}
    {}
};

