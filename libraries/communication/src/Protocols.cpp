#include "communication/Protocols.h"
#include "communication/ProtoIGTL.h"
#include "utils/Overloading.h"

namespace curan {
namespace communication {
		
std::function<void(Client*)> get_interface(callable callable_type) {
	std::function<void(Client*)> val;
	std::visit(utils::overloaded{
		[&val](interface_igtl arg) { val = protocols::igtlink::start; },
		[&val](interface_empty arg) {val = [](Client* val) {}; } }, callable_type);
	return val;
	}

}
}
