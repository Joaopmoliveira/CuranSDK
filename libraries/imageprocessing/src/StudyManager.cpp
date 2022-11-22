#include "PkLibrary.h"

namespace curan {
	namespace image {

		void StudyManager::GetStudies(std::map<uint64_t, Study>& previews)
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

		StudyManager* StudyManager::Get()
		{
			static StudyManager proc{};
			return &proc;
		}

		uint32_t StudyManager::identifier = 0;

		Result StudyManager::LoadStudies(std::vector<std::filesystem::path> paths) {
			std::lock_guard guard{ mut };
			for (auto received_path : paths) {
				ImageIOType::Pointer gdcmImageOI = ImageIOType::New();
				using ReaderType = itk::ImageSeriesReader< itk::Image<ShortPixelType, Dimension3D>>;
				ReaderType::Pointer reader = ReaderType::New();
				reader->SetImageIO(gdcmImageOI);

				bool is_directory = std::filesystem::is_directory(received_path);

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
					nameGenerator->SetDirectory(received_path.u8string());

					std::vector<std::string> seriesUID = nameGenerator->GetSeriesUIDs();

					for (auto series_ident : seriesUID) {
						std::vector<std::string> filenames = nameGenerator->GetFileNames(series_ident);
						Study study;
						for (auto filename : filenames) {
							reader->SetFileName(filename);
							InternalImageType::Pointer image2D = InternalImageType::New();
							itk::CastImageFilter<itk::Image<ShortPixelType, Dimension3D>, InternalImageType>::Pointer filter = itk::CastImageFilter<itk::Image<ShortPixelType, Dimension3D>, InternalImageType>::New();

							itk::RescaleIntensityImageFilter<itk::Image<ShortPixelType, Dimension3D>, itk::Image<ShortPixelType, Dimension3D>>::Pointer rescale = itk::RescaleIntensityImageFilter<itk::Image<ShortPixelType, Dimension3D>, itk::Image<ShortPixelType, Dimension3D>>::New();
							rescale->SetInput(reader->GetOutput());
							rescale->SetOutputMinimum(0);
							rescale->SetOutputMaximum(itk::NumericTraits<CharPixelType>::max());

							filter->SetInput(rescale->GetOutput());
							image2D = filter->GetOutput();
							try {
								image2D->Update();
							}
							catch (const itk::ExceptionObject& e) {
								std::string s{ e.what() };
								curan::utils::console->info("Exception thrown: " + s);
								return Result::PK_PATH_FAILURE;
							}
							study.study_img.push_back(image2D);
						}
						const DictionaryType& dictionary = gdcmImageOI->GetMetaDataDictionary();

						auto end = dictionary.End();

						std::string entryId = "0010|0010";
						auto tagItr = dictionary.Find(entryId);

						if (tagItr == end)
						{
							std::cerr << "Tag " << entryId; std::cerr << " not found in the DICOM header" << std::endl;
							return Result::PK_FAILED_TO_FIND_BACKEND;
						}

						MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType*>(tagItr->second.GetPointer());
						std::string tagvalue;
						if (entryvalue) {
							tagvalue = entryvalue->GetMetaDataObjectValue();
						}
						else {
							std::cerr << "Entry was not of string type" << std::endl;
							return Result::PK_FAILED_TO_FIND_BACKEND;
						}

						study.individual_name = (tagvalue.size() < 14) ? tagvalue : tagvalue.substr(0, 14);
						int image_preview_index = study.study_img.size() / 2;
						auto preview_image = study.study_img[image_preview_index];

						// we need to build the preview which will be used by other portions of the code
						SkAlphaType alpha_channel = kOpaque_SkAlphaType;
						SkColorType color_type = kGray_8_SkColorType;
						auto size = preview_image->GetLargestPossibleRegion().GetSize();
						study.individual_name = tagvalue;

						if (study.study_img.size() != 1)
							study.image_number = std::to_string(study.study_img.size()) + " Images";
						else
							study.image_number = std::to_string(study.study_img.size()) + " Image";

						SkImageInfo information = SkImageInfo::Make(size[0], size[1], color_type, alpha_channel);
						SkPixmap image_data = SkPixmap(information, preview_image->GetBufferPointer(), size[0]);
						study.image = SkImage::MakeRasterCopy(image_data);

						study_container.emplace(identifier, study);

						identifier += 1;
					}
				}
				else {
					//If we have a single file then we should upload it as a 
					//single study. The problem with this approach is that
					//we then need to check if there is a study currently 
					//in the map.. this is an unsolved problem.
					reader->SetFileName(received_path.u8string());

					InternalImageType::Pointer image2D = InternalImageType::New();
					itk::CastImageFilter<itk::Image<ShortPixelType, Dimension3D>, InternalImageType>::Pointer filter = itk::CastImageFilter<itk::Image<ShortPixelType, Dimension3D>, InternalImageType>::New();

					itk::RescaleIntensityImageFilter<itk::Image<ShortPixelType, Dimension3D>, itk::Image<ShortPixelType, Dimension3D>>::Pointer rescale = itk::RescaleIntensityImageFilter<itk::Image<ShortPixelType, Dimension3D>, itk::Image<ShortPixelType, Dimension3D>>::New();
					rescale->SetInput(reader->GetOutput());
					rescale->SetOutputMinimum(0);
					rescale->SetOutputMaximum(itk::NumericTraits<CharPixelType>::max());

					filter->SetInput(rescale->GetOutput());

					filter->SetInput(rescale->GetOutput());
					image2D = filter->GetOutput();
					try {
						image2D->Update();
					}
					catch (const itk::ExceptionObject& e) {
						return Result::PK_PATH_FAILURE;
					}

					Study study;
					study.study_img.push_back(image2D);
					const DictionaryType& dictionary = gdcmImageOI->GetMetaDataDictionary();

					auto end = dictionary.End();

					std::string entryId = "0010|0010";
					auto tagItr = dictionary.Find(entryId);

					if (tagItr == end)
					{
						std::cerr << "Tag " << entryId; std::cerr << " not found in the DICOM header" << std::endl;
						return Result::PK_FAILED_TO_FIND_BACKEND;
					}

					MetaDataStringType::ConstPointer entryvalue = dynamic_cast<const MetaDataStringType*>(tagItr->second.GetPointer());
					std::string tagvalue;
					if (entryvalue) {
						tagvalue = entryvalue->GetMetaDataObjectValue();
					}
					else {
						std::cerr << "Entry was not of string type" << std::endl;
						return Result::PK_FAILED_TO_FIND_BACKEND;
					}

					study.individual_name = (tagvalue.size() < 14) ? tagvalue : tagvalue.substr(0, 14);
					int image_preview_index = study.study_img.size() / 2;
					auto preview_image = study.study_img[image_preview_index];

					// we need to build the preview which will be used by other portions of the code
					SkAlphaType alpha_channel = kOpaque_SkAlphaType;
					SkColorType color_type = kGray_8_SkColorType;
					auto size = preview_image->GetLargestPossibleRegion().GetSize();
					study.individual_name = tagvalue;

					if (study.study_img.size() != 1)
						study.image_number = std::to_string(study.study_img.size()) + " Images";
					else
						study.image_number = std::to_string(study.study_img.size()) + " Image";

					SkImageInfo information = SkImageInfo::Make(size[0], size[1], color_type, alpha_channel);
					SkPixmap image_data = SkPixmap(information, preview_image->GetBufferPointer(), size[0]);
					study.image = SkImage::MakeRasterCopy(image_data);

					study_container.emplace(identifier, study);
					identifier += 1;
				}
			}
			return Result::PK_SUCCESS;
		}

		Result StudyManager::UnloadStudy(uint64_t identifier)
		{
			std::lock_guard guard{ mut };
			auto it = study_container.find(identifier);
			if (it == study_container.end()) {// if true it means we did not find a match and we should responde with a message signaling the failure
				return Result::PK_VOLUME_NOT_PRESENT;
			}
			else {
				study_container.erase(it);
				return Result::PK_SUCCESS;
			}
		}

		Result StudyManager::GetStudy(uint64_t identifier, Study& out_vol)
		{
			std::lock_guard guard{ mut };
			auto it = study_container.find(identifier);
			if (it == study_container.end()) {
				return Result::PK_VOLUME_NOT_PRESENT;
			}
			else {
				out_vol = it->second;
				return Result::PK_SUCCESS;
			}
		}
	}
}