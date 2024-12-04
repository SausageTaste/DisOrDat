#include <iostream>
#include <thread>

#include <spdlog/spdlog.h>
#include <asio.hpp>
#include <sung/general/bytes.hpp>


namespace {

    enum class PduType {
        other = 0,
        entity_state = 1,
        fire = 2,
        detonation = 3,
        collision = 4,
        service_request = 5,
        resupply_offer = 6,
        resupply_received = 7,
        resupply_cancel = 8,
        repair_complete = 9,
        repair_response = 10,
        create_entity = 11,
        remove_entity = 12,
        start_resume = 13,
        stop_freeze = 14,
        acknowledge = 15,
        action_request = 16,
        action_response = 17,
        data_query = 18,
        set_data = 19,
        data = 20,
        event_report = 21,
        comment = 22,
        electronic_emission = 23,
        designator = 24,
        transmitter = 25,
        signal = 26,
        receiver = 27,
        announce_object = 129,
        delete_object = 130,
        describe_application = 131,
        describe_event = 132,
        describe_object = 133,
        request_event = 134,
        request_object = 135,
        time_space_position_indicator_fi = 140,
        appearance_fi = 141,
        articulated_parts_fi = 142,
        fire_fi = 143,
        detonation_fi = 144,
        point_object_state = 150,
        linear_object_state = 151,
        areal_object_state = 152,
        environment = 153,
        transfer_control_request = 155,
        transfer_control = 156,
        transfer_control_acknowledge = 157,
        intercom_control = 160,
        intercom_signal = 161,
        aggregate = 170,
    };

    const char* to_str(PduType pdu_type) {
        switch (pdu_type) {
            case PduType::other:
                return "other";
            case PduType::entity_state:
                return "entity_state";
            case PduType::fire:
                return "fire";
            case PduType::detonation:
                return "detonation";
            case PduType::collision:
                return "collision";
            case PduType::service_request:
                return "service_request";
            case PduType::resupply_offer:
                return "resupply_offer";
            case PduType::resupply_received:
                return "resupply_received";
            case PduType::resupply_cancel:
                return "resupply_cancel";
            case PduType::repair_complete:
                return "repair_complete";
            case PduType::repair_response:
                return "repair_response";
            case PduType::create_entity:
                return "create_entity";
            case PduType::remove_entity:
                return "remove_entity";
            case PduType::start_resume:
                return "start_resume";
            case PduType::stop_freeze:
                return "stop_freeze";
            case PduType::acknowledge:
                return "acknowledge";
            case PduType::action_request:
                return "action_request";
            case PduType::action_response:
                return "action_response";
            case PduType::data_query:
                return "data_query";
            case PduType::set_data:
                return "set_data";
            case PduType::data:
                return "data";
            case PduType::event_report:
                return "event_report";
            case PduType::comment:
                return "comment";
            case PduType::electronic_emission:
                return "electronic_emission";
            case PduType::designator:
                return "designator";
            case PduType::transmitter:
                return "transmitter";
            case PduType::signal:
                return "signal";
            case PduType::receiver:
                return "receiver";
            case PduType::announce_object:
                return "announce_object";
            case PduType::delete_object:
                return "delete_object";
            case PduType::describe_application:
                return "describe_application";
            case PduType::describe_event:
                return "describe_event";
            case PduType::describe_object:
                return "describe_object";
            case PduType::request_event:
                return "request_event";
            case PduType::request_object:
                return "request_object";
            case PduType::time_space_position_indicator_fi:
                return "time_space_position_indicator_fi";
            case PduType::appearance_fi:
                return "appearance_fi";
            case PduType::articulated_parts_fi:
                return "articulated_parts_fi";
            case PduType::fire_fi:
                return "fire_fi";
            case PduType::detonation_fi:
                return "detonation_fi";
            case PduType::point_object_state:
                return "point_object_state";
            case PduType::linear_object_state:
                return "linear_object_state";
            case PduType::areal_object_state:
                return "areal_object_state";
            case PduType::environment:
                return "environment";
            case PduType::transfer_control_request:
                return "transfer_control_request";
            case PduType::transfer_control:
                return "transfer_control";
            case PduType::transfer_control_acknowledge:
                return "transfer_control_acknowledge";
            case PduType::intercom_control:
                return "intercom_control";
            case PduType::intercom_signal:
                return "intercom_signal";
            case PduType::aggregate:
                return "aggregate";
        }
        return "unknown";
    }


    struct PduHeader {
        PduType pdu_type() const { return static_cast<PduType>(pdu_type_); }

        const char* pdu_type_str() const { return ::to_str(this->pdu_type()); }

        uint8_t version_;
        uint8_t exercise_id_;
        uint8_t pdu_type_;
        uint8_t protocol_family_;
        sung::BEValue<uint32_t> timestamp_;
        sung::BEValue<uint16_t> length_;
        sung::BEValue<uint16_t> padding_;
    };


    struct SimAdress {
        sung::BEValue<uint16_t> site_id_;
        sung::BEValue<uint16_t> app_id_;
    };


    struct EnttId {
        SimAdress sim_addr_;
        sung::BEValue<uint16_t> entt_id_;
    };


    struct FixedDatum {
        sung::BEValue<uint32_t> datum_id_;
        sung::BEValue<uint32_t> datum_value_;
    };


    struct VariableDatum {
        sung::BEValue<uint32_t> datum_id_;
        sung::BEValue<uint32_t> datum_length_;
        uint8_t* datum_;
    };


    struct EnttType {
        uint8_t kind_;
        uint8_t domain_;
        sung::BEValue<uint16_t> country_;
        uint8_t category_;
        uint8_t subcategory_;
        uint8_t specific_;
        uint8_t extra_;
    };


    struct LinearVelVector {
        sung::BEValue<float> x_;
        sung::BEValue<float> y_;
        sung::BEValue<float> z_;
    };


    struct WorldCoord {
        sung::BEValue<double> x_;
        sung::BEValue<double> y_;
        sung::BEValue<double> z_;
    };


    struct EulerAngles {
        sung::BEValue<float> psi_;
        sung::BEValue<float> theta_;
        sung::BEValue<float> phi_;
    };


    struct DataPdu {
        PduHeader header_;
        EnttId originating_entt_;
        EnttId receiving_entt_;
        sung::BEValue<uint32_t> request_id_;
        sung::BEValue<uint32_t> padding_;
        sung::BEValue<uint32_t> num_of_fixed_datum_;
        sung::BEValue<uint32_t> num_of_variable_datum_;
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
        std::stringstream ss;

        for (size_t i = 0; i < size; ++i) {
            if (i % line_size == 0) {
                if (i != 0)
                    std::cout << '\n';

                for (size_t j = 0; j < indent_size; ++j) std::cout << " ";
            }

            const auto byte = *reinterpret_cast<const uint8_t*>(data + i);
            ss << fmt::format("{:02X} ", byte);
        }

        SPDLOG_INFO(ss.str());
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

            std::stringstream ss;

            ss << fmt::format("\nBytes ({})\n", bytes_transferred);
            //::print_bytes(recv_buffer_.data(), bytes_transferred, 16, 2);

            if (bytes_transferred < sizeof(::PduHeader)) {
                ss << fmt::format(
                    "Received bytes is less than PDU header size\n"
                );
                return this->start_recv();
            }

            const auto pdu_header = reinterpret_cast<::PduHeader*>(
                recv_buffer_.data()
            );
            ss << fmt::format(
                "PDU Header: version={}, exercise_id={}, pdu_type={}, "
                "protocol_family={}, timestamp={}, length={}\n",
                pdu_header->version_,
                pdu_header->exercise_id_,
                pdu_header->pdu_type_str(),
                pdu_header->protocol_family_,
                pdu_header->timestamp_.get(),
                pdu_header->length_.get()
            );

            if (pdu_header->pdu_type_ == 1) {
                ss << fmt::format("Entity State PDU\n");

                const auto entt_pdu = reinterpret_cast<::EnttPdu*>(
                    recv_buffer_.data()
                );

                ss << fmt::format(
                    "  entt_id={}:{}:{}, force_id={}, "
                    "num_of_articulation_param={}\n",
                    entt_pdu->entt_id_.sim_addr_.site_id_.get(),
                    entt_pdu->entt_id_.sim_addr_.app_id_.get(),
                    entt_pdu->entt_id_.entt_id_.get(),
                    entt_pdu->force_id_,
                    entt_pdu->num_of_articulation_param_
                );

                ss << fmt::format(
                    "  kind={}, domain={}, country={}, category={}, "
                    "subcategory={}, specific={}, extra={}\n",
                    entt_pdu->entt_type_.kind_,
                    entt_pdu->entt_type_.domain_,
                    entt_pdu->entt_type_.country_.get(),
                    entt_pdu->entt_type_.category_,
                    entt_pdu->entt_type_.subcategory_,
                    entt_pdu->entt_type_.specific_,
                    entt_pdu->entt_type_.extra_
                );

                ss << fmt::format(
                    "  alt_kind={}, alt_domain={}, alt_country={}, "
                    "alt_category={}, alt_subcategory={}, alt_specific={}, "
                    "alt_extra={}\n",
                    entt_pdu->alt_entt_type_.kind_,
                    entt_pdu->alt_entt_type_.domain_,
                    entt_pdu->alt_entt_type_.country_.get(),
                    entt_pdu->alt_entt_type_.category_,
                    entt_pdu->alt_entt_type_.subcategory_,
                    entt_pdu->alt_entt_type_.specific_,
                    entt_pdu->alt_entt_type_.extra_
                );

                ss << fmt::format(
                    "  linear_vel={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
                    entt_pdu->entt_linear_vel_.x_.get(),
                    entt_pdu->entt_linear_vel_.y_.get(),
                    entt_pdu->entt_linear_vel_.z_.get()
                );

                ss << fmt::format(
                    "  loc={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
                    entt_pdu->entt_loc_.x_.get(),
                    entt_pdu->entt_loc_.y_.get(),
                    entt_pdu->entt_loc_.z_.get()
                );

                ss << fmt::format(
                    "  orient={{psi={:.2f}, theta={:.2f}, "
                    "phi={:.2f}}}\n",
                    entt_pdu->entt_orient_.psi_.get(),
                    entt_pdu->entt_orient_.theta_.get(),
                    entt_pdu->entt_orient_.phi_.get()
                );
            } else if (pdu_header->pdu_type_ == 20) {
                ss << fmt::format("Data PDU\n");

                const auto data_pdu = reinterpret_cast<::DataPdu*>(
                    recv_buffer_.data()
                );

                ss << fmt::format(
                    "  originating_entt={}:{}:{}\n",
                    data_pdu->originating_entt_.sim_addr_.site_id_.get(),
                    data_pdu->originating_entt_.sim_addr_.app_id_.get(),
                    data_pdu->originating_entt_.entt_id_.get()
                );

                ss << fmt::format(
                    "  receiving_entt={}:{}:{}\n",
                    data_pdu->receiving_entt_.sim_addr_.site_id_.get(),
                    data_pdu->receiving_entt_.sim_addr_.app_id_.get(),
                    data_pdu->receiving_entt_.entt_id_.get()
                );

                ss << fmt::format(
                    "  request_id={}, padding={}, num_of_fixed_datum={}, "
                    "num_of_variable_datum={}\n",
                    data_pdu->request_id_.get(),
                    data_pdu->padding_.get(),
                    data_pdu->num_of_fixed_datum_.get(),
                    data_pdu->num_of_variable_datum_.get()
                );

                const auto fixed_data_num = data_pdu->num_of_fixed_datum_.get();
                const auto fixed_data = reinterpret_cast<::FixedDatum*>(
                    recv_buffer_.data() + sizeof(::DataPdu)
                );
                ss << fmt::format("Fixed data ({})\n", fixed_data_num);
                for (size_t i = 0; i < fixed_data_num; ++i) {
                    ss << fmt::format(
                        "  id={}, value={}\n",
                        fixed_data[i].datum_id_.get(),
                        fixed_data[i].datum_value_.get()
                    );
                }
            }

            SPDLOG_INFO(ss.str().substr(0, ss.str().size() - 1));
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
    std::thread th([&] { io_context.run(); });

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        SPDLOG_INFO("Running...");
    }

    return 0;
}
