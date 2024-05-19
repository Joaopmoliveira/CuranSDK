#ifndef CURAN_STUDY_MANAGER_HEADER_FILE_
#define CURAN_STUDY_MANAGER_HEADER_FILE_

#include <mutex>
#include <filesystem>
#include <map>
#include <string>
#include "ImageProcessingDefinitions.h"

namespace curan{
    namespace image{

		using callback_removed = std::function<void(void)>;
		using callback_added = std::function<void(void)>;

		class StudyManager {
		public:

			bool load_studies(std::vector<std::filesystem::path> paths);

			bool unload_study(uint64_t identifier);

			bool get_study(uint64_t identifier, Study& out_vol);

			void get_studies(std::map<uint64_t, Study>& previews);

			template <typename... T>
			void for_each(T &&...u)
			{
				std::for_each(study_container.begin(),study_container.end(),std::forward<T>(u)...);
			}

		private:
			static uint32_t identifier;

			StudyManager();

			~StudyManager();

			std::map<uint64_t, Study> study_container;

			std::mutex mut;
		};
    }
}

#endif