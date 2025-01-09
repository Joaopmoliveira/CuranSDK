#include "communication/Client.h"
#include "communication/Server.h"
#include <asio.hpp>
#include "communication/ProtocolValidationHelper.h"
#include "utils/TheadPool.h"
#include <numeric>
#include <csignal>
#include <iostream>

struct DummyProtocol;

/*
In memory the following message with be stored with the following memory layout
  <-  8 bytes ->        <-  size_of_vector ->
[ size_of_vector ]     [    ... data ....    ]
*/
class DummyMessage
{
    std::vector<double> m_values;
    size_t vector_size;
    std::vector<unsigned char> m_memory;

    friend DummyProtocol;

public:
    DummyMessage(const std::vector<double> &vals) : vector_size{0}
    {
        m_values = vals;
        vector_size = vals.size();
        m_memory.resize(sizeof(vector_size) + m_values.size() * sizeof(double));
    }

    DummyMessage() : vector_size{0}
    {
        m_memory.resize(sizeof(vector_size));
    }

    void set_vector(const std::vector<double> &vals)
    {
        m_values = vals;
        vector_size = vals.size();
        m_memory.resize(sizeof(vector_size) + m_values.size() * sizeof(double));
    }

    inline unsigned char *danger_get_buffer()
    {
        return m_memory.data();
    }

    inline unsigned char *danger_get_body_buffer()
    {
        return m_memory.data() + sizeof(size_t);
    }

    void deserialize_header()
    {
        assert(m_memory.size() >= sizeof(size_t));
        std::memcpy(&vector_size, get_buffer(), get_header_size());
        m_values.resize(vector_size);
        m_memory.resize(vector_size * sizeof(double) + sizeof(size_t));
    }

    void deserialize()
    {
        assert(m_memory.size() >= get_body_size() + get_header_size());
        std::memcpy(m_values.data(), get_buffer() + get_header_size(), get_body_size());
    }

    void serialize()
    {
        assert(vector_size > 1);
        m_memory.resize(get_body_size() + get_header_size());
        std::memcpy(m_memory.data(), &vector_size, sizeof(size_t));
        std::memcpy(m_memory.data() + sizeof(size_t), m_values.data(), m_values.size() * sizeof(double));
    }

    std::vector<double> get_vector() const
    {
        return m_values;
    }

    inline const unsigned char *get_buffer()
    {
        return m_memory.data();
    }

    inline const unsigned char *get_body_buffer()
    {
        return m_memory.data() + sizeof(size_t);
    }

    inline size_t buffer_size() const
    {
        return m_memory.size();
    }

    size_t get_header_size() const
    {
        return sizeof(size_t);
    }

    size_t get_body_size() const
    {
        return m_values.size() * sizeof(double);
    }
};

struct DummyClientConnection
{
    std::shared_ptr<DummyMessage> message = nullptr;
    std::shared_ptr<curan::communication::Client<DummyProtocol>> owner;
    DummyClientConnection(std::shared_ptr<curan::communication::Client<DummyProtocol>> supplied_owner) : owner{supplied_owner}
    {
    }
};

struct DummyProtocol
{
public:
    using signature = std::function<void(const size_t &, const std::error_code &, std::shared_ptr<DummyMessage>)>;

    static void start(std::shared_ptr<curan::communication::Client<DummyProtocol>> client)
    {
        std::cout << "started service" << std::endl;
        DummyClientConnection val{client};
        val.message = std::make_shared<DummyMessage>();
        read_header_first_time(std::move(val));
    }

    void static read_header_first_time(DummyClientConnection val)
    {
        std::cout << "reading header: " << val.message->m_memory.size() << std::endl;
        asio::async_read(val.owner->get_socket().get_underlying_socket(),
                         asio::buffer(val.message->danger_get_buffer(), val.message->get_header_size()), asio::transfer_all(),
                         [val](std::error_code ec, std::size_t len)
                         {
                             std::cout << "received header" << std::endl;
                             if (!ec)
                             {
                                 read_body(val, ec);
                             }
                             else
                             {
                                 val.owner->transverse_callables(0, ec, val.message);
                                 val.owner->get_socket().close();
                             }
                         });
    }

    void static read_body(DummyClientConnection val, std::error_code ec)
    {
        std::cout << "reading body" << std::endl;
        val.message->deserialize_header();

        asio::async_read(val.owner->get_socket().get_underlying_socket(),
                         asio::buffer(val.message->danger_get_body_buffer(), val.message->get_body_size()), asio::transfer_all(),
                         [val](std::error_code ec, std::size_t len)
                         {
		if (!ec)
			read_header(val, ec);
		else {
			val.owner->transverse_callables(0, ec, val.message);
			val.owner->get_socket().close();
		} });
    }

    void static read_header(DummyClientConnection val, std::error_code ec)
    {
        // we have a message fully unpacked in memory that we must broadcast to all
        // listeners of the interface. We do this by calling the templated broadcast method
        size_t temp = 0;
        val.message->deserialize();
        val.owner->transverse_callables(temp, ec, val.message);
        val.message = std::make_shared<DummyMessage>();
        asio::async_read(val.owner->get_socket().get_underlying_socket(),
                         asio::buffer(val.message->danger_get_buffer(), val.message->get_header_size()), asio::transfer_all(),
                         [val](std::error_code ec, std::size_t len)
                         {
		if (!ec) {
			read_body(val, ec);
		} else {
			val.owner->transverse_callables(0, ec, val.message);
			val.owner->get_socket().close();
		} });
    }
};

auto vector_message_with_pseudo_random_size()
{
    static size_t initial_size = 1;
    ++initial_size;
    std::vector<double> to_send;
    to_send.resize(initial_size % 10 + 5);
    std::iota(to_send.begin(), to_send.end(), 1);
    auto msg = std::make_shared<DummyMessage>(to_send);
    msg->serialize();
    auto msg_to_send = curan::utilities::CaptureBuffer::make_shared(msg->danger_get_buffer(), msg->get_body_size() + msg->get_header_size(), msg);
    return std::make_tuple(msg_to_send, to_send);
}

asio::io_context io_context;

void signal_handler(int signal)
{
    io_context.stop();
}

int function_to_test_message()
{
    std::signal(SIGINT, signal_handler);
    std::vector<double> number;
    number.resize(10);
    std::iota(number.begin(), number.end(), 1);
    DummyMessage data{number};
    std::printf("data to send:\n");
    for (auto v : data.get_vector())
        std::printf(" %f", v);
    std::printf("\n");
    data.serialize();
    std::vector<unsigned char> buffer;
    buffer.reserve(data.buffer_size());
    std::memcpy(buffer.data(), data.get_buffer(), data.buffer_size());

    DummyMessage data_after;
    std::memcpy(data_after.danger_get_buffer(), buffer.data(), data_after.get_header_size());
    data_after.deserialize_header();
    std::memcpy(data_after.danger_get_body_buffer(), buffer.data() + data_after.get_header_size(), data_after.get_body_size());
    data_after.deserialize();
    std::printf("data received:\n");
    for (auto v : data_after.get_vector())
        std::printf(" %f", v);
    std::printf("\n");

    auto [message, random_vector_generated] = vector_message_with_pseudo_random_size();
    std::printf("random genenrated data to send:\n");
    for (auto v : random_vector_generated)
        std::printf(" %f", v);
    std::printf("\n");

    DummyMessage data_after_from_random;
    std::memcpy(data_after_from_random.danger_get_buffer(), message->begin()->data(), data_after_from_random.get_header_size());
    data_after_from_random.deserialize_header();
    std::memcpy(data_after_from_random.danger_get_body_buffer(), (unsigned char *)message->begin()->data() + data_after_from_random.get_header_size(), data_after_from_random.get_body_size());
    data_after_from_random.deserialize();
    std::printf("random data received:\n");
    for (auto v : data_after_from_random.get_vector())
        std::printf(" %f", v);
    std::printf("\n");
    return 0;
}

int function_to_call()
{
    unsigned short port = 50000;
    asio::ip::tcp::resolver resolver(io_context);
    auto pool = curan::utilities::ThreadPool::create(1);

    auto server = curan::communication::Server<DummyProtocol>::make(io_context,
                                                                    port,
                                                                    [](std::error_code ec)
                                                                    {
                                                                        if (ec)
                                                                        {
                                                                            std::cout << "error has occured" << std::endl;
                                                                            return false;
                                                                        }
                                                                        else
                                                                        {
                                                                            std::cout << "received client" << std::endl;
                                                                            return true;
                                                                        }
                                                                    });

    pool->submit("server sender", [&]()
                 {

        while(!io_context.stopped()){
            
            auto [message,random_vector_generated] = vector_message_with_pseudo_random_size(); 
            if(server->write(message))
                std::printf("sent message from server\n"); 
            else 
                std::printf("did not send message\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
        } });


    auto client = curan::communication::Client<DummyProtocol>::make(io_context,
                                                                    resolver.resolve("localhost", "50000"),
                                                                    std::chrono::milliseconds(1000),
                                                                    [](std::error_code e)
                                                                    {
                                                                        std::cout << "the connection was? ";
                                                                        if (e){
                                                                            std::cout << "unsuccefull: " << e.message() << std::endl;
                                                                            std::raise(SIGINT);
                                                                        } else {
                                                                            std::cout << "succeful" << std::endl;
                                                                        }
                                                                    });

    client->connect([](const size_t &, const std::error_code &, std::shared_ptr<DummyMessage> msg){ 
        auto received_vector = msg->get_vector(); 
        std::printf("received vector of size: %llu\nwith values: ",received_vector.size()); 
        for(auto v : received_vector) 
            std::printf("%f ",v); 
    });

    auto work = asio::make_work_guard(io_context);
    io_context.run();
    return 0;
}

int main()
{
    try
    {
        return function_to_call();
    }
    catch (std::runtime_error &e)
    {
        std::cout << "exception:" << e.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "generic exception" << std::endl;
    }
}