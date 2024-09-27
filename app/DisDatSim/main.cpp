#include <bitset>
#include <iostream>

#include <spdlog/spdlog.h>
#include <asio.hpp>
#include <sung/general/bytes.hpp>


namespace {

    struct PduHeader {
        uint8_t version_;
        uint8_t exercise_id_;
        uint8_t pdu_type_;
        uint8_t protocol_family_;
        uint32_t timestamp_;
        uint16_t length_;
        uint16_t padding_;
    };


    template <typename T>
    T flip_byte_order(T value) {
        T result = 0;
        for (size_t i = 0; i < sizeof(T); ++i) {
            result <<= 8;
            result |= value & 0xFF;
            value >>= 8;
        }
        return result;
    }


    class UdpServer {

    public:
        UdpServer(asio::io_context& io_context)
            : socket_(
                  io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), 3000)
              ) {
            this->start_recv();
        }

    private:
        void start_recv() {
            socket_.async_receive_from(
                asio::buffer(recv_buffer_),
                remote_endpoint_,
                std::bind(
                    &UdpServer::handle_recv,
                    this,
                    asio::placeholders::error,
                    asio::placeholders::bytes_transferred
                )
            );
        }

        void handle_recv(
            const std::error_code& error, size_t bytes_transferred
        ) {
            if (error) {
                std::cerr << "Recv error: " << error.message() << std::endl;
                return;
            }

            const auto line_count = bytes_transferred / 16;

            fmt::print("\nBytes ({})\n", bytes_transferred);
            for (size_t i = 0; i < line_count; ++i) {
                fmt::print(" ");
                for (size_t j = 0; j < 16; ++j) {
                    fmt::print(
                        " {:02x}",
                        static_cast<unsigned char>(recv_buffer_[i * 16 + j])
                    );
                }
                fmt::print("\n");
            }

            if (bytes_transferred >= sizeof(::PduHeader)) {
                const auto pdu_header = reinterpret_cast<::PduHeader*>(
                    recv_buffer_.data()
                );

                fmt::print(
                    "PDU Header: version={}, exercise_id={}, pdu_type={}, "
                    "protocol_family={}, timestamp={}, length={}\n",
                    pdu_header->version_,
                    pdu_header->exercise_id_,
                    pdu_header->pdu_type_,
                    pdu_header->protocol_family_,
                    ::flip_byte_order(pdu_header->timestamp_),
                    ::flip_byte_order(pdu_header->length_)
                );
            }

            this->start_recv();
        }

        void handle_send(
            std::shared_ptr<std::string> /*message*/,
            const std::error_code& /*error*/,
            std::size_t /*bytes_transferred*/
        ) {}

        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint remote_endpoint_;
        std::array<char, 1024 * 8> recv_buffer_;
    };

}  // namespace


int main() {
    asio::io_context io_context;
    ::UdpServer server(io_context);
    io_context.run();

    return 0;
}
