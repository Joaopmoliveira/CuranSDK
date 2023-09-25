---
layout: "page"
permalink : "/communication/"
---

### Communication

The communication library is probably the hardest library to undestand out of all our libraries currently. Thats because we use heavily asyncronous code which is always hard to make sure that no bugs exist in our code. To link the Communcation library in your executable this is how you procede in cmake. 

```cmake
add_executable(myexecutable main.cpp)

target_link_libraries(myexecutable PUBLIC
communication
)
```

This automatically links against the necessary third_party libraries as needed. Now lets go and look into some code. As you have already seen we have emplyed threads throught the previous examples, and we went to great lenghts to guarantee  that we wait for threadsafe flags, as in the chapter [Utilities](/utilities/). Now we will employ a similar tought process. The library which we base ourselfs on is called ASIO (Asyncrounous Input and Output). This is one of the great marvels of C++ in my opinion. 

Asio works based on a very important object, the 'asio::io_context'. This object is used to syncronize communcation calls and so much more that I cannot describe all the ways in which is used. In curan, to simplify your life while trying to use custom procotols we designed a class which is 

```cpp
int main() {
try {
	unsigned short port = 50000;
	asio::io_context io_context;
	curan::communication::interface_igtl igtlink_interface;

    asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));

	curan::communication::Client::Info construction{ io_context,igtlink_interface };
	construction.endpoints = endpoints;
	curan::communication::Client client{ construction };
	auto connectionstatus = client.connect(bar);
	auto val = io_context.run();
	curan::utilities::cout << "stopped running";
	laucher.join();
}
catch (std::exception& e) {
	curan::utilities::cout << "CLient exception was thrown"+std::string(e.what());
	return 1;
}
return 0;
}
```

## Add your own custom protocol to the library

Once you start developing a larger applications, the need arises to deal with third-party protocols, e.g. DICOM, which we would need to 
