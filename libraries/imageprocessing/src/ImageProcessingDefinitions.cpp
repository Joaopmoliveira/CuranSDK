#include "imageprocessing/ImageProcessingDefinitions.h"

namespace curan{
namespace image{

std::optional<Volume> Study::try_convert_to_volume(){
    return std::nullopt;
}

}
}