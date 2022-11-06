#include "Definitions.h"
#include <mutex>
#include <sigslot/signal.hpp>
#include <filesystem>
#include <map>
#include <string>

namespace curan{
    namespace image{
        		/*
		The PkImageManager internally contains
		a map pointing towards ITK images, where
		each image is associated with a number
		which must be used to obtain a pointer
		towards said image.
		*/
		class StudyManager {
		public:

			/*
			The get method returns the application wide study manager
			which can be used in different ways depending on the wishes of the 
			developer.
			*/
			static StudyManager* Get();

			/*
			* The loaded study call member is a collection of slots which 
			whish to be warned when a study is loaded into memory. Whenever thats
			the case, the submited method will be called and it will return an
			integer with the index of the new study which in turn can be used to
			query for that particular study.
			*/
			sigslot::signal<int*> loaded_study_call;

			/*
			The PkLoadVolume function receives a pointer to the
			PkImageManager and a reference to a PkLoadVolumeInfo
			structure. It searches internally for the volumes already
			in memory and makes sure that the provided identifier is
			unique. If it's not a PkResult with an error is returned.
			*/
			Result LoadStudies(std::vector<std::filesystem::path> paths);

			/*
			The PkUnloadVolume function removes a given image from
			memory. If to much memory has been consumed by the
			application one can erase images from the Image
			manager to solve the problem.
			*/
			Result UnloadStudy(uint64_t identifier);

			/*
			The PkGetVolume function returns a PkVolume stored
			inside the Image manager associated with a given
			identifier.
			*/
			Result GetStudy(uint64_t identifier, Study& out_vol);

			/*
			Obtain a pointer to the vector containing the 
			previews of all the current loaded studies.
			*/
			void GetStudies(std::map<uint64_t, Study>& previews);

		private:
			static uint32_t identifier;

			StudyManager();

			~StudyManager();

			std::map<uint64_t, Study> study_container;

			std::mutex mut;
		};
    }
}