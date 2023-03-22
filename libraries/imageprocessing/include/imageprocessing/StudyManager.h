#include <mutex>
#include <filesystem>
#include <map>
#include <string>
#include "ImageProcessingDefinitions.h"

namespace curan{
    namespace image{

		class StudyManager {
		public:

			bool load_studies(std::vector<std::filesystem::path> paths);

			bool unload_study(uint64_t identifier);

			bool get_study(uint64_t identifier, Study& out_vol);

			void get_studies(std::map<uint64_t, Study>& previews);

		private:
			static uint32_t identifier;

			StudyManager();

			~StudyManager();

			std::map<uint64_t, Study> study_container;

			std::mutex mut;
		};
    }
}