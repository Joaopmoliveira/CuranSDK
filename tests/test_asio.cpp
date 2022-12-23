#include <asio.hpp>
#include "utils\Callable.h"

class session : public std::enable_shared_from_this<session>
{
    session(asio::ip::tcp::socket&& socket, asio::io_context& io_context) : io_context_(io_context), socket_(std::move(socket))
    {
    };

    session(asio::ip::tcp::endpoint& desired_endpoint, asio::io_context& io_context)  : io_context_(io_context), socket_(io_context_)
    {
    };

public:
    static std::shared_ptr<session> make_shared(asio::ip::tcp::socket&& socket, asio::io_context& io_context) {
        return std::move(std::make_shared<session>(socket,io_context));
    }

    static std::shared_ptr<session> make_shared(asio::ip::tcp::endpoint& desired_endpoint, asio::io_context& io_context) {
        return std::move(std::make_shared<session>(desired_endpoint,io_context));
    }

    std::shared_ptr<session> shared_copy() {
        return shared_from_this();
    }

    void start()
    {
        do_read();
    }

private:
    void do_read()
    {
        auto self(shared_copy());
        socket_.async_read_some(asio::buffer(data_, max_length),
            [this, self](std::error_code ec, std::size_t length)
            {
                if (!ec)
                {
                    do_write(length);
                }
            });
    }

    void do_write(std::size_t length)
    {
        auto self(shared_from_this());
        asio::async_write(socket_, asio::buffer(data_, length),
            [this, self](std::error_code ec, std::size_t /*length*/)
            {
                if (!ec)
                {
                    do_read();
                }
            });
    }

    asio::io_context& io_context_;
    asio::ip::tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};

class server : public std::enable_shared_from_this<server> , curan::utils::Callable<std::shared_ptr<session>>
{

    server(asio::io_context& io_context, short port)
        : io_context_(io_context), acceptor_(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
        socket_(io_context)
    {
        do_accept();
    }
public:
    static std::shared_ptr<server> make_shared(asio::io_context& io_context, short port) {
        return std::make_shared<server>(io_context,port);
    }

    std::shared_ptr<server> shared_copy() {
        return shared_from_this();
    }

private:
    void do_accept()
    {
        acceptor_.async_accept(socket_,
            [this](std::error_code ec)
            {
                if (!ec)
                {
                    std::shared_ptr<session> ptr = session::make_shared(std::move(socket_), io_context_);
                }

                do_accept();
            });
    }

    asio::ip::tcp::acceptor acceptor_;
    asio::ip::tcp::socket socket_;
    asio::io_context& io_context_;
};

int main()
{
    asio::io_context cxt;
    auto communication_blocking_thread = [&cxt]() {
        asio::executor_work_guard<asio::io_context::executor_type> word_guard{ cxt.get_executor()};
        cxt.run();
        //this will block for ever
    };

    auto serv = server::make_shared(cxt, 50000);
    

    //now we lauch the thread which actually contains the number we are interested in
    std::thread communication_thread{ communication_blocking_thread };
}
