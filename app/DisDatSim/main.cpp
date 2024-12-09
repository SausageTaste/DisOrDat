#include <chrono>
#include <iostream>
#include <thread>
#include <unordered_map>

#include <spdlog/spdlog.h>
#include <asio.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <sung/general/angle.hpp>
#include <sung/general/bytes.hpp>
#include <sung/general/time.hpp>


namespace {

    using Vec3 = glm::dvec3;
    using Angle = sung::TAngle<double>;

    const auto CENTER_TO_PRIME_MERIDIAN = Vec3{ 1, 0, 0 };
    const auto CENTER_TO_ASIA = Vec3{ 0, 1, 0 };
    const auto CENTER_TO_NORTH = Vec3{ 0, 0, 1 };


    class SimpleFixedWing {

    public:
        SimpleFixedWing()
            : quat_(1, 0, 0, 0), pos_(-3049328.82, 4049133.27, 3859976.86) {}

        void update(double dt) {
            const auto anti_gravity_n = glm::normalize(pos_);

            // Adjust roll
            {
                const auto dst_entt_left_n = glm::normalize(
                    glm::cross(anti_gravity_n, this->entt_front())
                );
                const auto align = glm::dot(dst_entt_left_n, this->entt_up());
                quat_ = glm::rotate(quat_, align * dt, this->entt_front());
            }

            // Adjust elevation
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_front());
                quat_ = glm::rotate(quat_, align * dt, this->entt_right());
            }

            // Head forward
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_up());
                if (align > 0.999)
                    pos_ += this->entt_front() * 100000.0 * dt;
            }
        }

        Vec3 make_eular() const {
            const auto eular = glm::eulerAngles(quat_);
            return Vec3(eular.z, eular.y, eular.x);
        }

        Vec3 entt_front() const { return quat_ * CENTER_TO_PRIME_MERIDIAN; }
        Vec3 entt_up() const { return quat_ * (-CENTER_TO_NORTH); }
        Vec3 entt_right() const { return quat_ * (CENTER_TO_ASIA); }

        glm::dquat quat_;  // Quaternion
        Vec3 pos_;         // Geocentric position
        bool to_north = true;
    };

}  // namespace


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


    class PduHeader {

    public:
        PduType pdu_type() const { return static_cast<PduType>(pdu_type_); }

        const char* pdu_type_str() const { return ::to_str(this->pdu_type()); }

        PduHeader& set_default() {
            version_ = 7;
            exercise_id_ = 1;
            pdu_type_ = 0;
            protocol_family_ = 5;
            timestamp_.set(sung::get_time_unix());
            length_ = 0;
            padding_ = 0;
            return *this;
        }

        PduHeader& set_type(PduType pdu_type) {
            pdu_type_ = static_cast<uint8_t>(pdu_type);
            return *this;
        }

        PduHeader& set_len(uint16_t len) {
            length_ = len;
            return *this;
        }

        uint8_t version_;
        uint8_t exercise_id_;
        uint8_t pdu_type_;
        uint8_t protocol_family_;
        sung::BEValue<uint32_t> timestamp_;
        sung::BEValue<uint16_t> length_;
        sung::BEValue<uint16_t> padding_;
    };
    static_assert(8 * sizeof(PduHeader) == 96);


    struct SimAdress {
        sung::BEValue<uint16_t> site_id_;
        sung::BEValue<uint16_t> app_id_;
    };


    class EnttId {

    public:
        uint16_t site_id() const { return sim_addr_.site_id_.get(); }
        uint16_t app_id() const { return sim_addr_.app_id_.get(); }
        uint16_t entt_id() const { return entt_id_.get(); }

        std::string to_str() const {
            return fmt::format(
                "{}:{}:{}", this->site_id(), this->app_id(), this->entt_id()
            );
        }

        EnttId& set(uint16_t site_id, uint16_t app_id, uint16_t entt_id) {
            sim_addr_.site_id_.set(site_id);
            sim_addr_.app_id_.set(app_id);
            entt_id_.set(entt_id);
            return *this;
        }

    private:
        SimAdress sim_addr_;
        sung::BEValue<uint16_t> entt_id_;
    };
    static_assert(8 * sizeof(EnttId) == 48);


    struct FixedDatum {
        sung::BEValue<uint32_t> datum_id_;
        sung::BEValue<uint32_t> datum_value_;
    };


    struct VariableDatum {
        sung::BEValue<uint32_t> datum_id_;
        sung::BEValue<uint32_t> datum_length_;
        uint8_t* datum_;
    };


    class EnttType {

    public:
        EnttType& set(
            uint8_t kind,
            uint8_t domain,
            uint16_t country,
            uint8_t category,
            uint8_t subcategory,
            uint8_t specific,
            uint8_t extra
        ) {
            kind_ = kind;
            domain_ = domain;
            country_.set(country);
            category_ = category;
            subcategory_ = subcategory;
            specific_ = specific;
            extra_ = extra;
            return *this;
        }

        uint8_t kind_;
        uint8_t domain_;
        sung::BEValue<uint16_t> country_;
        uint8_t category_;
        uint8_t subcategory_;
        uint8_t specific_;
        uint8_t extra_;
    };
    static_assert(8 * sizeof(EnttType) == 64);


    struct LinearVelVector {
        LinearVelVector& set(float x, float y, float z) {
            x_.set(x);
            y_.set(y);
            z_.set(z);
            return *this;
        }

        sung::BEValue<float> x_;
        sung::BEValue<float> y_;
        sung::BEValue<float> z_;
    };
    static_assert(8 * sizeof(LinearVelVector) == 96);


    struct WorldCoord {
        WorldCoord& set(double x, double y, double z) {
            x_.set(x);
            y_.set(y);
            z_.set(z);
            return *this;
        }

        WorldCoord& set(const Vec3& v) {
            x_.set(v.x);
            y_.set(v.y);
            z_.set(v.z);
            return *this;
        }

        sung::BEValue<double> x_;
        sung::BEValue<double> y_;
        sung::BEValue<double> z_;
    };
    static_assert(8 * sizeof(WorldCoord) == 192);


    /**
     * On VR-Forces, the rest orientation is such that top face of an entity is
     * towards along the north pole axis, front direction is towards from center
     * to the Greenwich observatory.
     */
    struct EulerAngles {
        EulerAngles& clear() {
            psi_.set(0);
            theta_.set(0);
            phi_.set(0);
            return *this;
        }

        EulerAngles& set(
            const Angle& psi, const Angle& theta, const Angle& phi
        ) {
            psi_.set(psi.rad());
            theta_.set(theta.rad());
            phi_.set(phi.rad());
            return *this;
        }

        EulerAngles& set(const Vec3& radians) {
            psi_.set(radians.x);
            theta_.set(radians.y);
            phi_.set(radians.z);
            return *this;
        }

        // Psi is the heading angle.
        EulerAngles& set_head(const Angle& psi) {
            psi_.set(psi.rad());
            return *this;
        }

        // Theta is the elevation angle.
        EulerAngles& set_elev(const Angle& theta) {
            theta_.set(theta.rad());
            return *this;
        }

        // Phi is the bank angle.
        EulerAngles& set_roll(const Angle& phi) {
            phi_.set(phi.rad());
            return *this;
        }

        sung::BEValue<float> psi_;
        sung::BEValue<float> theta_;
        sung::BEValue<float> phi_;
    };
    static_assert(8 * sizeof(EulerAngles) == 96);


    struct EnttAppearance {
        EnttAppearance& clear() {
            general_apperaance_.set(0);
            specific_appearance_varient_.set(0);
            return *this;
        }

        sung::BEValue<uint16_t> general_apperaance_;
        sung::BEValue<uint16_t> specific_appearance_varient_;
    };
    static_assert(8 * sizeof(EnttAppearance) == 32);


    struct DeadReckoningParam {
        DeadReckoningParam& set_default() {
            algorithm_ = 1;
            other_params_.fill(0);
            linear_acc_.set(0, 0, 0);
            angular_vel_.set(0, 0, 0);
            return *this;
        }

        uint8_t algorithm_;
        std::array<uint8_t, 15> other_params_;
        LinearVelVector linear_acc_;
        LinearVelVector angular_vel_;
    };
    static_assert(8 * sizeof(DeadReckoningParam) == 320);


    struct EnttMarking {
        EnttMarking& clear() {
            charset_ = 0;
            str_.fill(0);
            return *this;
        }

        uint8_t charset_;
        std::array<uint8_t, 11> str_;
    };
    static_assert(8 * sizeof(EnttMarking) == 96);


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
        EnttAppearance entt_appearance_;
        DeadReckoningParam dead_reckoning_param_;
        EnttMarking entt_marking_;
        sung::BEValue<uint32_t> entt_capabilities_;
        std::array<uint8_t, 128 / 8> padding_;
    };
    static_assert(8 * sizeof(EnttPdu) == 1280);


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


    class ClientRecord {

    public:
        double elapsed_since_comm() const {
            return std::min(last_recv_.elapsed(), last_send_.elapsed());
        }

        sung::MonotonicRealtimeTimer last_recv_;
        sung::MonotonicRealtimeTimer last_send_;
    };


    std::string to_str(asio::ip::udp::endpoint ep) {
        return fmt::format("{}:{}", ep.address().to_string(), ep.port());
    }


    class ClientManager {

    public:
        void update() {
            for (auto it = data_.begin(); it != data_.end();) {
                if (it->second.elapsed_since_comm() > 10) {
                    SPDLOG_INFO("Client timed out: {}", ::to_str(it->first));
                    it = data_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        ClientRecord& notify_recv(const asio::ip::udp::endpoint& ep) {
            auto& client = this->get_or_create(ep);
            client.last_recv_.check();
            return client;
        }

        ClientRecord& notify_send(const asio::ip::udp::endpoint& ep) {
            auto& client = this->get_or_create(ep);
            client.last_send_.check();
            return client;
        }

        void print_all() const {
            SPDLOG_INFO("Clients {}", data_.size());

            for (const auto& [ep, record] : data_) {
                SPDLOG_INFO(
                    "  - {}:{}: recv={}, send={}",
                    ep.address().to_string(),
                    ep.port(),
                    record.last_recv_.elapsed(),
                    record.last_send_.elapsed()
                );
            }
        }

        auto begin() { return data_.begin(); }
        auto end() { return data_.end(); }

    private:
        ClientRecord& get_or_create(const asio::ip::udp::endpoint& ep) {
            auto it = data_.find(ep);
            if (it == data_.end()) {
                it = data_.emplace(ep, ClientRecord{}).first;
                SPDLOG_INFO("Client added: {}", ::to_str(ep));
            }

            return it->second;
        }

        std::unordered_map<asio::ip::udp::endpoint, ClientRecord> data_;
    };


    class UdpRadioTower {

    public:
        UdpRadioTower(asio::io_context& io_context)
            : io_context_(io_context)
            , socket_(io_context)
            , endpoint_(
                  asio::ip::address_v4::from_string("127.255.255.255"), 3000
              )
            , tick_timer_(io_context, std::chrono::milliseconds(5000)) {
            std::error_code error;
            socket_.open(asio::ip::udp::v4(), error);

            if (error) {
                SPDLOG_ERROR("Failed to open socket: {}", error.message());
                return;
            }

            socket_.set_option(asio::ip::udp::socket::reuse_address(true));
            socket_.set_option(asio::socket_base::broadcast(true));

            timer_.check();
            this->start_tick();
        }

        ~UdpRadioTower() {
            std::error_code error;
            socket_.close(error);

            if (error) {
                SPDLOG_ERROR("Failed to close socket: {}", error.message());
            }
        }

    private:
        void start_tick() {
            tick_timer_ = asio::steady_timer(
                io_context_, std::chrono::milliseconds(200)
            );
            tick_timer_.async_wait(std::bind(&UdpRadioTower::tick, this));
        }

        void tick() {
            const auto dt = timer_.check_get_elapsed();
            fixed_wing_.update(dt);

            EnttPdu pdu;
            pdu.header_.set_default()
                .set_type(::PduType::entity_state)
                .set_len(sizeof(EnttPdu));
            pdu.entt_id_.set(1, 3003, 1001);
            pdu.force_id_ = 3;
            pdu.num_of_articulation_param_ = 0;
            pdu.entt_type_.set(1, 2, 45, 1, 7, 0, 0);
            pdu.alt_entt_type_.set(1, 1, 225, 1, 1, 1, 0);
            pdu.entt_linear_vel_.set(0, 0, 0);
            pdu.entt_loc_.set(fixed_wing_.pos_);
            pdu.entt_orient_.set(fixed_wing_.make_eular());
            pdu.entt_appearance_.clear();
            pdu.dead_reckoning_param_.set_default();
            pdu.entt_marking_.clear();
            pdu.entt_capabilities_.set(0);
            pdu.padding_.fill(0);

            socket_.send_to(asio::buffer(&pdu, sizeof(pdu)), endpoint_);
            SPDLOG_INFO(
                "Sent to {}: len={}, type={}",
                ::to_str(endpoint_),
                pdu.header_.length_.get(),
                pdu.header_.pdu_type_str()
            );

            this->start_tick();
        }

        asio::io_context& io_context_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint endpoint_;
        asio::steady_timer tick_timer_;
        sung::MonotonicRealtimeTimer timer_;
        ::SimpleFixedWing fixed_wing_;
    };


    class UdpServer {

    public:
        UdpServer(asio::io_context& io_context)
            : socket_(
                  io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), 3000)
              )
            , tick_timer_(io_context, std::chrono::milliseconds(1000 / 30)) {
            tick_timer_.async_wait(std::bind(&UdpServer::tick, this));
            this->start_recv();
        }

    private:
        void tick() {
            tick_timer_.async_wait(std::bind(&UdpServer::tick, this));
            return;
        }

        void start_recv() {
            socket_.async_receive_from(
                asio::buffer(recv_buffer_),
                remote_ep_tmp_,
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

            clients_.notify_recv(remote_ep_tmp_);
            clients_.update();

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

            SPDLOG_INFO(
                "Received from {}: size={}, type={}",
                ::to_str(remote_ep_tmp_),
                bytes_transferred,
                pdu_header->pdu_type_str()
            );

            if (pdu_header->pdu_type_ == 1) {
                send_buffer_.clear();
                send_buffer_.push_back(
                    asio::buffer(recv_buffer_.data(), bytes_transferred)
                );

                ss << fmt::format("Entity State PDU\n");

                const auto entt_pdu = reinterpret_cast<::EnttPdu*>(
                    recv_buffer_.data()
                );

                ss << fmt::format(
                    "  entt_id={}, force_id={}, num_of_articulation_param={}\n",
                    entt_pdu->entt_id_.to_str(),
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
                    "  originating_entt={}\n",
                    data_pdu->originating_entt_.to_str()
                );

                ss << fmt::format(
                    "  receiving_entt={}\n", data_pdu->receiving_entt_.to_str()
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
            const std::error_code& error, std::size_t bytes_transferred
        ) {
            if (error) {
                SPDLOG_ERROR(
                    "Send error: {} ({})",
                    error.message(),
                    std::hash<std::thread::id>{}(std::this_thread::get_id())
                );
            } else {
                SPDLOG_INFO(
                    "Sent {} bytes ({})",
                    bytes_transferred,
                    std::hash<std::thread::id>{}(std::this_thread::get_id())
                );
            }
        }

        asio::ip::udp::socket socket_;
        asio::steady_timer tick_timer_;
        asio::ip::udp::endpoint remote_ep_tmp_;
        ClientManager clients_;
        std::array<char, 1024 * 8> recv_buffer_;
        std::vector<asio::const_buffer> send_buffer_;
    };

}  // namespace


int main() {
    asio::io_context io_context;
    ::UdpServer server(io_context);
    std::thread th([&] { io_context.run(); });

    ::UdpRadioTower radio_tower(io_context);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
