#ifndef CURAN_IMAGE_DEFIITIONS_HEADER_FILE_
#define CURAN_IMAGE_DEFIITIONS_HEADER_FILE_

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
		using short_pixel_type = unsigned short;
		using char_pixel_type = unsigned char;

		constexpr unsigned int Dimension3D = 3;
		constexpr unsigned int Dimension2D = 2;
		using input_DICOM_image_type = itk::Image<short_pixel_type, Dimension3D>;

		using Internal2DImageType = itk::Image<char_pixel_type, Dimension2D>;
		using InternalImageType = itk::Image<char_pixel_type, Dimension3D>;
		using RescaleType = itk::RescaleIntensityImageFilter<input_DICOM_image_type, input_DICOM_image_type>;
		using FilterType = itk::CastImageFilter<input_DICOM_image_type, InternalImageType>;
		using ImageIOType = itk::GDCMImageIO;
		using SlicerType = itk::ResampleImageFilter<InternalImageType, InternalImageType>;
		using IterateType = itk::ExtractImageFilter<InternalImageType, InternalImageType>;
		using DictionaryType = itk::MetaDataDictionary;
		using MetaDataStringType = itk::MetaDataObject<std::string>;

namespace reconstruction{
		constexpr int INPUT_COMPONENTS = 1;
		constexpr uint64_t ACCUMULATION_MULTIPLIER = 256;
		constexpr uint64_t ACCUMULATION_MAXIMUM = 65535;
		constexpr uint64_t ACCUMULATION_THRESHOLD = 65279; // calculate manually to possibly save on computation time
		constexpr double fraction1_256 = 1.0 / 256;
		constexpr double fraction255_256 = 255.0 / 256;
}

		struct Study {
			std::vector<InternalImageType::Pointer> study_img;
			std::string individual_name;
			std::string image_number;
		};

		struct Volume {
			std::string individual_name;
			InternalImageType::Pointer volume;
		};

		struct Image {
			Internal2DImageType::Pointer image;
		};

		namespace reconstruction {
			enum Interpolation{
				NEAREST_NEIGHBOR_INTERPOLATION,
				LINEAR_INTERPOLATION
			};

			enum Compounding{
				UNDEFINED_COMPOUNDING_MODE,
				LATEST_COMPOUNDING_MODE,
				MAXIMUM_COMPOUNDING_MODE,
				MEAN_COMPOUNDING_MODE,
			};

			enum FillingStrategy
			{
				GAUSSIAN,
				GAUSSIAN_ACCUMULATION,
				DISTANCE_WEIGHT_INVERSE,
				NEAREST_NEIGHBOR,
				STICK
			};
		}
    }
}

#endif