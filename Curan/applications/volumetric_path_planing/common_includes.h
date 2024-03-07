#ifndef COMMON_INCLUDES_HEADER
#define COMMON_INCLUDES_HEADER

#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/IconResources.h"
#include "userinterface/widgets/definitions/Interactive.h"
#include "userinterface/widgets/Button.h"
#include "userinterface/widgets/TextBlob.h"
#include "userinterface/widgets/SliderPanel.h"
#include "userinterface/widgets/MiniPage.h"
#include "userinterface/widgets/ItemExplorer.h"
#include "userinterface/widgets/Overlay.h"
#include "userinterface/widgets/MutatingTextPanel.h"
#include <unordered_map>
#include <optional>
#include <charconv>
#include <functional>
#include "utils/Job.h"
#include "utils/TheadPool.h"

#include "userinterface/widgets/Drawable.h"
#include "utils/Lockable.h"
#include "userinterface/widgets/SignalProcessor.h"
#include "userinterface/widgets/ImageWrapper.h"
#include "userinterface/widgets/ComputeImageBounds.h"
#include "utils/Overloading.h"

#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkImageFileWriter.h"

#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"

#include <Eigen/Dense>

using DicomPixelType = unsigned short;
using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension>;

#endif