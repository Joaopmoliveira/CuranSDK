#include "imageprocessing/StudyManager.h"
#include "utils/Logger.h"

namespace curan {
namespace image {

void StudyManager::get_studies(std::map<uint64_t, Study>& previews)
{
	std::lock_guard guard{ mut };
	previews = study_container;
}

StudyManager::StudyManager()
{
}

StudyManager::~StudyManager()
{
}

uint32_t StudyManager::identifier = 0;

bool StudyManager::load_studies(std::vector<std::filesystem::path> paths) {
	std::lock_guard guard{ mut };
	for (auto received_path : paths) {
		ImageIOType::Pointer gdcmImageOI = ImageIOType::New();
		using ReaderType = itk::ImageSeriesReader< itk::Image<short_pixel_type, Dimension3D>>;
		ReaderType::Pointer reader = ReaderType::New();
		reader->SetImageIO(gdcmImageOI);

		//bool is_directory = std::filesystem::is_directory(received_path);

		if (std::filesystem::is_directory(received_path))
		{
		//If we have multiple files we need to process 
		//the information in a different way, namelly
		//we need to run through all the different studies
		//in the directory and load each image individually 
		//to a vector of dicom images which are related by  
		//the same series identifier (a single study)
		using NamesGeneratorType = itk::GDCMSeriesFileNames;
		NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();
		nameGenerator->SetUseSeriesDetails(true);
		nameGenerator->AddSeriesRestriction("0008|0021");
		nameGenerator->SetDirectory(received_path.string());

		std::vector<std::string> seriesUID = nameGenerator->GetSeriesUIDs();

		for (auto series_ident : seriesUID) {
			std::vector<std::string> filenames = nameGenerator->GetFileNames(series_ident);
			Study study;
			for (auto filename : filenames) {
				reader->SetFileName(filename);
				InternalImageType::Pointer image2D = InternalImageType::New();
				itk::CastImageFilter<itk::Image<short_pixel_type, Dimension3D>, InternalImageType>::Pointer filter = itk::CastImageFilter<itk::Image<short_pixel_type, Dimension3D>, InternalImageType>::New();

				itk::RescaleIntensityImageFilter<itk::Image<short_pixel_type, Dimension3D>, itk::Image<short_pixel_type, Dimension3D>>::Pointer rescale = itk::RescaleIntensityImageFilter<itk::Image<short_pixel_type, Dimension3D>, itk::Image<short_pixel_type, Dimension3D>>::New();
				rescale->SetInput(reader->GetOutput());
				rescale->SetOutputMinimum(0);
				rescale->SetOutputMaximum(itk::NumericTraits<char_pixel_type>::max());

				filter->SetInput(rescale->GetOutput());
				image2D = filter->GetOutput();
				try {
					image2D->Update();
				}
				catch (const itk::ExceptionObject& e) {
					std::string s{ e.what() };
					utilities::print<utilities::minor_failure>("StudyManager load_studies exception thrown: {}",e.what());
					return false;
				}
				study.study_img.push_back(image2D);
			}
			const DictionaryType& dictionary = gdcmImageOI->GetMetaDataDictionary();

			auto end = dictionary.End();

			std::string entryId = "0010|0010";
			auto tagItr = dictionary.Find(entryId);

			if (tagItr == end)
			{
				utilities::print<utilities::minor_failure>("Tag {} not found in the DICOM header\n",entryId);
				return false;
			}

			MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType*>(tagItr->second.GetPointer());
			std::string tagvalue;
			if (entryvalue) {
				tagvalue = entryvalue->GetMetaDataObjectValue();
			}
			else {
				utilities::print<utilities::minor_failure>("Entry was not of string type\n");
				return false;
			}

			study.individual_name = (tagvalue.size() < 14) ? tagvalue : tagvalue.substr(0, 14);
			int image_preview_index = study.study_img.size() / 2;
			auto preview_image = study.study_img[image_preview_index];

			study.individual_name = tagvalue;

			if (study.study_img.size() != 1)
				study.image_number = std::to_string(study.study_img.size()) + " Images";
			else
				study.image_number = std::to_string(study.study_img.size()) + " Image";

			study_container.emplace(identifier, study);

			identifier += 1;
		}
		} else {
		//If we have a single file then we should upload it as a 
		//single study. The problem with this approach is that
		//we then need to check if there is a study currently 
		//in the map.. this is an unsolved problem.
		reader->SetFileName(received_path.string());

		InternalImageType::Pointer image2D = InternalImageType::New();
		itk::CastImageFilter<itk::Image<short_pixel_type, Dimension3D>, InternalImageType>::Pointer filter = itk::CastImageFilter<itk::Image<short_pixel_type, Dimension3D>, InternalImageType>::New();

		itk::RescaleIntensityImageFilter<itk::Image<short_pixel_type, Dimension3D>, itk::Image<short_pixel_type, Dimension3D>>::Pointer rescale = itk::RescaleIntensityImageFilter<itk::Image<short_pixel_type, Dimension3D>, itk::Image<short_pixel_type, Dimension3D>>::New();
		rescale->SetInput(reader->GetOutput());
		rescale->SetOutputMinimum(0);
		rescale->SetOutputMaximum(itk::NumericTraits<char_pixel_type>::max());

		filter->SetInput(rescale->GetOutput());

		filter->SetInput(rescale->GetOutput());
		image2D = filter->GetOutput();
		try {
			image2D->Update();
		}
		catch (...) {
			return false;
		}

		Study study;
		study.study_img.push_back(image2D);
		const DictionaryType& dictionary = gdcmImageOI->GetMetaDataDictionary();

		auto end = dictionary.End();

		std::string entryId = "0010|0010";
		auto tagItr = dictionary.Find(entryId);

		if (tagItr == end)
		{
			utilities::print<utilities::minor_failure>("Tag {} not found in the DICOM header\n",entryId);
			return false;
		}

		MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType*>(tagItr->second.GetPointer());
		std::string tagvalue;
		if (entryvalue) {
			tagvalue = entryvalue->GetMetaDataObjectValue();
		} else {
			utilities::print<utilities::minor_failure>("Entry was not of string type\n");
			return false;
		}

		study.individual_name = (tagvalue.size() < 14) ? tagvalue : tagvalue.substr(0, 14);
		int image_preview_index = study.study_img.size() / 2;
		auto preview_image = study.study_img[image_preview_index];

		// we need to build the preview which will be used by other portions of the code
		study.individual_name = tagvalue;

		if (study.study_img.size() != 1)
			study.image_number = std::to_string(study.study_img.size()) + " Images";
		else
			study.image_number = std::to_string(study.study_img.size()) + " Image";

		study_container.emplace(identifier, study);
		identifier += 1;
		}
	}
	return true;
}

bool StudyManager::unload_study(uint64_t identifier)
{
	std::lock_guard guard{ mut };
	auto it = study_container.find(identifier);
	if (it == study_container.end()) {// if true it means we did not find a match and we should responde with a message signaling the failure
		return false;
	} else {
		study_container.erase(it);
		return true;
	}
}

bool StudyManager::get_study(uint64_t identifier, Study& out_vol)
{
	std::lock_guard guard{ mut };
	auto it = study_container.find(identifier);
	if (it == study_container.end()) {
		return false;
	} else {
		out_vol = it->second;
		return true;
	}
}

}
}