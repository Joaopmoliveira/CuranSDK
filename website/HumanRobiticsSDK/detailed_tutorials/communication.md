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

Asio works based on a very important object, the 'asio::io_context'. This object is used to syncronize communcation calls and so much more that I cannot describe all the ways in which is used.

Usually in this context you have both clients and servers communicating with eachother (in the case of TCP being the underlying protocol). 

## Client

A typical communication loop will start with the definition of our main function 

```cpp
int main() {
	unsigned short port = 50000;
	asio::io_context io_context;

	asio::ip::tcp::resolver resolver(io_context);
	auto endpoints = resolver.resolve("localhost", std::to_string(port));
```

where we define an ASIO context to deal with our assyncronous calls, we define a resolver to find the endpoint we wish to connect ourselfs t (in this case we want to connect to a server running in the local machine, thus the "localhost" ip and the port 50000)

```cpp
	curan::communication::interface_igtl igtlink_interface;
```

this is the hardest part to explain. Imagine you have two people, where both want to communicate between eachother. If they do not speak the same language, then its impossible for them to establish meaninfull communication. To solve this problem we define aprior the language that the server will use to communicate with us, the client. This interface contains a state machine which deals with any protocol specific calls whilst reading and writing these custom messages (more on this later). For now we will use the OpenIGTLink protocol to communicate between our two machines. 

Once we have defined the language we will use to communicate with the server we can create our client 

```cpp
	curan::communication::Client::Info construction{ io_context,igtlink_interface };
	construction.endpoints = endpoints;
	curan::communication::Client client{ construction };
	auto val = io_context.run();
	curan::utilities::cout << "stopped running";
	return 0;
}
```

this simple application does not do anything. In the next sections we will show how you can custimize your client behavior. 

## Customize your behavior 

Obviously you want to do things with the messages exchanged between the client and the server. To send a message from our client to our server you need a tool introduced in the [Utilities](/utilities/) library related with Memory blocks. Read the description of these classes before moving along in this tutorial. 

Assuming that you have read this description, we can implement 


## Server

## Customize your behavior 

## Add your own custom protocol to the library

Once you start developing a larger applications, the need arises to deal with third-party protocols, e.g. DICOM, which we would need to 
