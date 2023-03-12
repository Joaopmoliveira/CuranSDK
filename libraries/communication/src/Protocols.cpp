#include "communication/Protocols.h"
#include "communication/ProtoIGTL.h"

namespace supid_details {
	// helper type for the visitor #4
	template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
	// explicit deduction guide (not needed as of C++20)
	template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
}


namespace curan {
	namespace communication {
		std::function<void(Client*)> get_interface(callable callable_type) {
			std::function<void(Client*)> val;
			std::visit(supid_details::overloaded{
				[&val](interface_igtl arg) { val = protocols::igtlink::start; } }, callable_type);
			return val;
		}
	}
}
