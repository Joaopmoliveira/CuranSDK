#include "imageprocessing/KernelDescriptor.h"
namespace curan {
namespace image {
namespace reconstruction {

KernelDescriptor::KernelDescriptor()
{
}

KernelDescriptor::~KernelDescriptor()
{
	if (sticksList != nullptr)
		delete[] sticksList;

	if (kernel != nullptr)
		delete[] kernel;
}

KernelDescriptor::KernelDescriptor(const KernelDescriptor& other)
{
	stickLengthLimit = other.stickLengthLimit;
	numSticksToUse = other.numSticksToUse;    // the number of sticks to use in averaging the final voxel value
	numSticksInList = other.numSticksInList;         // the number of sticks in sticksList
	fillType = other.fillType;
	size = other.size;
	stdev = other.stdev;
	minRatio = other.minRatio;

	if (other.sticksList != nullptr) {
		if (sticksList != nullptr)
			delete[] sticksList;
		size_t sticksList_size = 39;
		sticksList = new int[sticksList_size];
		std::memcpy(sticksList, other.sticksList, sticksList_size);
	}

	if (other.kernel != nullptr) {
		if (kernel != nullptr)
			delete[] kernel;
		size_t kernel_size = size * size * size;
		kernel = new float[kernel_size];
		std::memcpy(kernel, other.kernel, kernel_size);
	}
}

KernelDescriptor& KernelDescriptor::operator=(const KernelDescriptor& other)
{
	if (this != &other) {
		stickLengthLimit = other.stickLengthLimit;
		numSticksToUse = other.numSticksToUse;    // the number of sticks to use in averaging the final voxel value
		numSticksInList = other.numSticksInList;         // the number of sticks in sticksList
		fillType = other.fillType;
		size = other.size;
		stdev = other.stdev;
		minRatio = other.minRatio;

		if (other.sticksList != nullptr) {
			if (sticksList != nullptr)
				delete[] sticksList;
			size_t sticksList_size = 39;
			sticksList = new int[sticksList_size];
			std::memcpy(sticksList, other.sticksList, sticksList_size);
		}

		if (other.kernel != nullptr) {
			if (kernel != nullptr)
				delete[] kernel;
			size_t kernel_size = size * size * size;
			kernel = new float[kernel_size];
			std::memcpy(kernel, other.kernel, kernel_size);
		}
	}
	return *this;
}

void KernelDescriptor::Allocate(){
	switch (fillType)
	{
	case reconstruction::FillingStrategy::DISTANCE_WEIGHT_INVERSE:
	{
		if (kernel != nullptr)
			delete[] kernel;
		kernel = new float[size * size * size];

		float range = (size - 1) / 2.;
		int index(0);
		for (int z = 0; z < size; z++)
		{
			for (int y = 0; y < size; y++)
			{
				for (int x = 0; x < size; x++)
				{
					float xD = (x - range);
					float yD = (y - range);
					float zD = (z - range);
					float distance = sqrt(xD * xD + yD * yD + zD * zD); // use euclidean distance
					if (distance) { // avoid division by zero
						float invD = fabsf(1.0f / distance);
						kernel[index] = invD;
					} else {
						kernel[index] = 0.0f; // center shouldn't have any weight anyway
					}
					index++;
				}
			}
		}

	}
	break;
	case reconstruction::FillingStrategy::GAUSSIAN:
	{
		if (kernel != nullptr)
			delete[] kernel;
		kernel = new float[size * size * size];

		float range = (size - 1) / 2.;
		const double Pi = 3.1415926535897932384626433832795;

		// divisors in the exponent
		float divisor = stdev * stdev * 2.0;

		// divisor in the non-exponent
		float termDivisor = pow(2 * Pi, 3. / 2) * pow(stdev, 3);

		int index(0);
		for (int z = 0; z < size; z++)
		{
			for (int y = 0; y < size; y++)
			{
				for (int x = 0; x < size; x++)
				{
					double xExp = pow((double)x - range, 2) / divisor;
					double yExp = pow((double)y - range, 2) / divisor;
					double zExp = pow((double)z - range, 2) / divisor;
					double eExp = -(xExp + yExp + zExp);

					float calcVal = (1.0f) / (termDivisor)*exp(eExp);
					kernel[index] = calcVal;
					index++;
				}
			}
		}
	}
	break;
	case reconstruction::FillingStrategy::GAUSSIAN_ACCUMULATION:
	{
		if (kernel != nullptr)
			delete[] kernel;
		kernel = new float[size * size * size];

		float range = (size - 1) / 2.;
		const double Pi = 3.1415926535897932384626433832795;

		// divisors in the exponent
		float divisor = stdev * stdev * 2.0;

		// divisor in the non-exponent
		float termDivisor = pow(2 * Pi, 3. / 2) * pow(stdev, 3);

		int index(0);
		for (int z = 0; z < size; z++)
		{
			for (int y = 0; y < size; y++)
			{
				for (int x = 0; x < size; x++)
				{
					double xExp = pow((double)x - range, 2) / divisor;
					double yExp = pow((double)y - range, 2) / divisor;
					double zExp = pow((double)z - range, 2) / divisor;
					double eExp = -(xExp + yExp + zExp);

					float calcVal = (1.0f) / (termDivisor)*exp(eExp);
					kernel[index] = calcVal;
					index++;
				}
			}
		}
	}
	break;
	case reconstruction::FillingStrategy::NEAREST_NEIGHBOR:
	
			// we do not need to allocate anything for the nearest neighbor algorithm
	break;
	case reconstruction::FillingStrategy::STICK:
	{
		numSticksInList = 13;
		if (sticksList != nullptr)
			delete[] sticksList;
		sticksList = new int[39];

		// 1x1, 2x0
		sticksList[0] = 1; sticksList[1] = 0; sticksList[2] = 0; // x, y, z
		sticksList[3] = 0; sticksList[4] = 1; sticksList[5] = 0;
		sticksList[6] = 0; sticksList[7] = 0; sticksList[8] = 1;

		// 2x1, 1x0
		sticksList[9] = 1; sticksList[10] = 1; sticksList[11] = 0;
		sticksList[12] = 1; sticksList[13] = 0; sticksList[14] = 1;
		sticksList[15] = 0; sticksList[16] = 1; sticksList[17] = 1;
		// 1x1, 1x-1, 1x0
		sticksList[18] = 1; sticksList[19] = -1; sticksList[20] = 0;
		sticksList[21] = 1; sticksList[22] = 0; sticksList[23] = -1;
		sticksList[24] = 0; sticksList[25] = 1; sticksList[26] = -1;

		// 3x1
		sticksList[27] = 1; sticksList[28] = 1; sticksList[29] = 1;
		// 2x1, 1x-1
		sticksList[30] = -1; sticksList[31] = 1; sticksList[32] = 1;
		sticksList[33] = 1; sticksList[34] = -1; sticksList[35] = 1;
		sticksList[36] = -1; sticksList[37] = -1; sticksList[38] = 1;

	}
	break;
	default:
		// should we return an error?
		break;
	}
}

}
}
}