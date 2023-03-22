#include "itkImage.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkResampleImageFilter.h"
#include "itkAffineTransform.h"
#include "itkExtractImageFilter.h"
#include "itkImageSliceConstIteratorWithIndex.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkImageLinearIteratorWithIndex.h"
#include "itkRegionOfInterestImageFilter.h"
#include "itkMetaDataObject.h"
#include "itkTransform.h"
#include "itkThresholdImageFilter.h"
#include "itkCannyEdgeDetectionImageFilter.h"

namespace curan{
    namespace image{
		using ShortPixelType = unsigned short;
		using CharPixelType = unsigned char;

		constexpr unsigned int Dimension3D = 3;
		using InputDICOMImageType = itk::Image<ShortPixelType, Dimension3D>;

		// The dimensions of the Internal image type are 3, 
		// not because it is required by the image itself,
		// which are 2D, but by the because it is the only
		// way to have the position of the 2D image of
		// individual pixels in 3D space. (This is required
		// to some algorithms we wish to implement such as 
		// volume reconstruction, volume mapping between 
		// two images and so on)
		using InternalImageType = itk::Image<CharPixelType, Dimension3D>;
		using RescaleType = itk::RescaleIntensityImageFilter<InputDICOMImageType, InputDICOMImageType>;
		using FilterType = itk::CastImageFilter<InputDICOMImageType, InternalImageType>;
		using ImageIOType = itk::GDCMImageIO;
		using SlicerType = itk::ResampleImageFilter<InternalImageType, InternalImageType>;
		using IterateType = itk::ExtractImageFilter<InternalImageType, InternalImageType>;
		using DictionaryType = itk::MetaDataDictionary;
		using MetaDataStringType = itk::MetaDataObject<std::string>;

		/*
		The PkStudy is the logical unit of the application.
		Once the user request to upload a given file or directory
		the software will identify all studies and load a vector
		containing a pointer for each 2D image.
		*/
		struct Study {
			std::vector<InternalImageType::Pointer> study_img;
			std::string individual_name;
			std::string image_number;
		};

		/*
		Wrapper around an ITK 3D volume which
		has a depth of 8bits per pixel.
		This volume must be reconstructed from
		the loaded study.
		*/
		struct Volume {
			std::string individual_name;
			InternalImageType::Pointer volume;
		};
    }
}