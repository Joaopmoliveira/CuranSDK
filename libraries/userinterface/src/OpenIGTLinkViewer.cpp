#include "userinterface/widgets/OpenIGTLinkViewer.h"

namespace curan {
	namespace ui {
		OpenIGTLinkViewer::OpenIGTLinkViewer(Info& info) {
		
		}

		std::shared_ptr<OpenIGTLinkViewer> OpenIGTLinkViewer::make(Info& info) {
			return std::make_shared<OpenIGTLinkViewer>(info);
		}

		void OpenIGTLinkViewer::process_message(igtl::MessageBase::Pointer pointer) {
		
		}

		drawablefunction OpenIGTLinkViewer::impldraw() {
			drawablefunction val;
			return val;
		}

		callablefunction OpenIGTLinkViewer::implcall() {
			callablefunction val;
			return val;
		}

		bool OpenIGTLinkViewer::interacts(double x, double y) {
			return true;
		}
	}
}