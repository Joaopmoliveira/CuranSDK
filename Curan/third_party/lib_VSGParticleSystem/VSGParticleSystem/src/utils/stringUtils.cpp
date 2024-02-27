#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {
void replaceAll(std::string &string, std::string const &old,
                std::string const &newString) {
    std::size_t positionFinded = string.find(old);
    if (positionFinded == std::string::npos)
        return;
    string.replace(positionFinded, old.length(), newString);
}
} // namespace vsgps
