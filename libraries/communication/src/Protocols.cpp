#include "communication/Protocols.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include "communication/ProtoProcHandler.h"
#include "utils/Overloading.h"

namespace curan {
namespace communication {
		
std::function<void(std::shared_ptr<Client>)> get_interface(callable callable_type) {
	std::function<void(std::shared_ptr<Client>)> val;
	std::visit(utilities::overloaded{
		[&val](interface_igtl arg) { val = protocols::igtlink::start; },
		[&val](interface_fri arg) { val = protocols::frimessage::start; },
		[&val](interface_prochandler arg) { val = protocols::proc_handler_message::start; },
		[&val](interface_empty arg) {val = [](std::shared_ptr<Client> val) {}; } }, callable_type);
	return val;
	}

}
}
