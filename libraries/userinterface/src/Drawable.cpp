#include "userinterface/widgets/Drawable.h"

namespace curan {
namespace ui {

Drawable::Drawable(SkRect size) : size{size} {

}

bool Drawable::is_leaf() {
	return true;
}

}
}