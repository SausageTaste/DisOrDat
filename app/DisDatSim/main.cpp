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


    struct SimAdress {
        uint16_t site_id_;
        uint16_t app_id_;
    };


    struct EnttId {
        SimAdress sim_addr_;
        uint16_t entt_id_;
    };


    struct FixedDatum {
        uint32_t datum_id_;
        uint32_t datum_value_;
    };


    struct VariableDatum {
        uint32_t datum_id_;
        uint32_t datum_length_;
        uint8_t* datum_;
    };


    struct EnttType {
        uint8_t kind_;
        uint8_t domain_;
        uint16_t country_;
        uint8_t category_;
        uint8_t subcategory_;
        uint8_t specific_;
        uint8_t extra_;
    };


    struct LinearVelVector {
        float x_;
        float y_;
        float z_;
    };


    struct WorldCoord {
        double x_;
        double y_;
        double z_;
    };


    struct EulerAngles {
        float psi_;
        float theta_;
        float phi_;
    };


    struct DataPdu {
        PduHeader header_;
        EnttId originating_entt_;
        EnttId receiving_entt_;
        uint32_t request_id_;
        uint32_t padding_;
        uint32_t num_of_fixed_datum_;
        uint32_t num_of_variable_datum_;
    };


    struct EnttPdu {
        PduHeader header_;
        EnttId entt_id_;
        uint8_t force_id_;
        uint8_t num_of_articulation_param_;
        EnttType entt_type_;
        EnttType alt_entt_type_;
        LinearVelVector entt_linear_vel_;
        WorldCoord entt_loc_;
        EulerAngles entt_orient_;
    };


    template <typename T>
    void print_bytes(
        const T* data, size_t size, size_t line_size, size_t indent_size
    ) {
        static_assert(sizeof(T) == 1, "sizeof(T) must be 1");

        for (size_t i = 0; i < size; ++i) {
            if (i % line_size == 0) {
                if (i != 0)
                    std::cout << '\n';

                for (size_t j = 0; j < indent_size; ++j) std::cout << " ";
            }

            const auto byte = *reinterpret_cast<const uint8_t*>(data + i);
            fmt::print("{:02X} ", byte);
        }
        std::cout << '\n';
    }

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

    template <>
    float flip_byte_order(float value) {
        uint32_t result = flip_byte_order(*reinterpret_cast<uint32_t*>(&value));
        return *reinterpret_cast<float*>(&result);
    }

    template <>
    double flip_byte_order(double value) {
        uint64_t result = flip_byte_order(*reinterpret_cast<uint64_t*>(&value));
        return *reinterpret_cast<double*>(&result);
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
            //::print_bytes(recv_buffer_.data(), bytes_transferred, 16, 2);

            if (bytes_transferred < sizeof(::PduHeader)) {
                fmt::print("Received bytes is less than PDU header size\n");
                return this->start_recv();
            }

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

            if (pdu_header->pdu_type_ == 1) {
                fmt::print("Entity State PDU\n");

                const auto entt_pdu = reinterpret_cast<::EnttPdu*>(
                    recv_buffer_.data()
                );

                fmt::print(
                    "  entt_id={}:{}:{}, force_id={}, "
                    "num_of_articulation_param={}\n",
                    ::flip_byte_order(entt_pdu->entt_id_.sim_addr_.site_id_),
                    ::flip_byte_order(entt_pdu->entt_id_.sim_addr_.app_id_),
                    ::flip_byte_order(entt_pdu->entt_id_.entt_id_),
                    entt_pdu->force_id_,
                    entt_pdu->num_of_articulation_param_
                );

                fmt::print(
                    "  kind={}, domain={}, country={}, category={}, "
                    "subcategory={}, specific={}, extra={}\n",
                    entt_pdu->entt_type_.kind_,
                    entt_pdu->entt_type_.domain_,
                    ::flip_byte_order(entt_pdu->entt_type_.country_),
                    entt_pdu->entt_type_.category_,
                    entt_pdu->entt_type_.subcategory_,
                    entt_pdu->entt_type_.specific_,
                    entt_pdu->entt_type_.extra_
                );

                fmt::print(
                    "  alt_kind={}, alt_domain={}, alt_country={}, "
                    "alt_category={}, alt_subcategory={}, alt_specific={}, "
                    "alt_extra={}\n",
                    entt_pdu->alt_entt_type_.kind_,
                    entt_pdu->alt_entt_type_.domain_,
                    ::flip_byte_order(entt_pdu->alt_entt_type_.country_),
                    entt_pdu->alt_entt_type_.category_,
                    entt_pdu->alt_entt_type_.subcategory_,
                    entt_pdu->alt_entt_type_.specific_,
                    entt_pdu->alt_entt_type_.extra_
                );

                fmt::print(
                    "  linear_vel={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
                    ::flip_byte_order(entt_pdu->entt_linear_vel_.x_),
                    ::flip_byte_order(entt_pdu->entt_linear_vel_.y_),
                    ::flip_byte_order(entt_pdu->entt_linear_vel_.z_)
                );

                fmt::print(
                    "  loc={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
                    ::flip_byte_order(entt_pdu->entt_loc_.x_),
                    ::flip_byte_order(entt_pdu->entt_loc_.y_),
                    ::flip_byte_order(entt_pdu->entt_loc_.z_)
                );

                fmt::print(
                    "  orient={{psi={:.2f}, theta={:.2f}, "
                    "phi={:.2f}}}\n",
                    ::flip_byte_order(entt_pdu->entt_orient_.psi_),
                    ::flip_byte_order(entt_pdu->entt_orient_.theta_),
                    ::flip_byte_order(entt_pdu->entt_orient_.phi_)
                );
            } else if (pdu_header->pdu_type_ == 20) {
                fmt::print("Data PDU\n");

                const auto data_pdu = reinterpret_cast<::DataPdu*>(
                    recv_buffer_.data()
                );

                fmt::print(
                    "  originating_entt={}:{}:{}\n",
                    ::flip_byte_order(
                        data_pdu->originating_entt_.sim_addr_.site_id_
                    ),
                    ::flip_byte_order(
                        data_pdu->originating_entt_.sim_addr_.app_id_
                    ),
                    ::flip_byte_order(data_pdu->originating_entt_.entt_id_)
                );

                fmt::print(
                    "  receiving_entt={}:{}:{}\n",
                    ::flip_byte_order(
                        data_pdu->receiving_entt_.sim_addr_.site_id_
                    ),
                    ::flip_byte_order(
                        data_pdu->receiving_entt_.sim_addr_.app_id_
                    ),
                    ::flip_byte_order(data_pdu->receiving_entt_.entt_id_)
                );

                fmt::print(
                    "  request_id={}, padding={}, num_of_fixed_datum={}, "
                    "num_of_variable_datum={}\n",
                    ::flip_byte_order(data_pdu->request_id_),
                    ::flip_byte_order(data_pdu->padding_),
                    ::flip_byte_order(data_pdu->num_of_fixed_datum_),
                    ::flip_byte_order(data_pdu->num_of_variable_datum_)
                );

                const auto fixed_data_count = ::flip_byte_order(
                    data_pdu->num_of_fixed_datum_
                );
                const auto fixed_data = reinterpret_cast<::FixedDatum*>(
                    recv_buffer_.data() + sizeof(::DataPdu)
                );
                fmt::print("Fixed data ({})\n", fixed_data_count);
                for (size_t i = 0; i < fixed_data_count; ++i) {
                    fmt::print(
                        "  id={}, value={}\n",
                        ::flip_byte_order(fixed_data[i].datum_id_),
                        ::flip_byte_order(fixed_data[i].datum_value_)
                    );
                }
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
