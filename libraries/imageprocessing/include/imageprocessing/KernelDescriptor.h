#ifndef CURAN_KERNEL_DESCRIPTOR_HEADER_FILE_
#define CURAN_KERNEL_DESCRIPTOR_HEADER_FILE_

#include "ImageProcessingDefinitions.h"

namespace curan {
	namespace image {
		namespace reconstruction {
            
			struct KernelDescriptor
			{
				//constructor
				KernelDescriptor();

				//destructor
				~KernelDescriptor();
				
				//copy constructor
				KernelDescriptor(const KernelDescriptor& other);

				//copy assignment
				KernelDescriptor& operator=(const KernelDescriptor& other);

				//move constructor

				//move assignment

				//data related with volume filling
				int stickLengthLimit= -1;
				int numSticksToUse= -1;    // the number of sticks to use in averaging the final voxel value
				int numSticksInList= -1;         // the number of sticks in sticksList
				
				reconstruction::FillingStrategy fillType = reconstruction::FillingStrategy::GAUSSIAN;
				int size = -1;
				float stdev = -1;
				float minRatio = -1;

				int* sticksList = nullptr; // triples each corresponding to a stick orientation
				float* kernel = nullptr; // stores the gaussian weights for this kernel


				void Allocate();
			};
        }
    }
}

#endif