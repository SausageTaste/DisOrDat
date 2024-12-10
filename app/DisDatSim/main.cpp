#include <chrono>
#include <iostream>
#include <set>
#include <thread>
#include <unordered_map>

#define SPDLOG_ACTIVE_LEVEL 0
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <asio.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <sung/general/time.hpp>

#include "disordat/pdu_struct.hpp"


namespace {

    int SITE_ID = 1;
    int APP_ID = 3003;

    const glm::dvec3 CENTER_TO_PRIME_MERIDIAN{ 1, 0, 0 };
    const glm::dvec3 CENTER_TO_ASIA{ 0, 1, 0 };
    const glm::dvec3 CENTER_TO_NORTH{ 0, 0, 1 };


    class Entity {

    public:
        virtual ~Entity() = default;

        virtual const glm::dvec3& pos() = 0;
        virtual const glm::dvec3& vel() = 0;
        virtual const glm::dvec3& acc() = 0;

        std::string name_;
        uint16_t dis_id_ = 0;
    };


    class PositionIntegrator {

    public:
        PositionIntegrator() : pos_(0), vel_(0), acc_(0) {}

        auto& pos() const { return pos_; }
        auto& vel() const { return vel_; }
        auto& acc() const { return acc_; }

        PositionIntegrator& set_pos(double x, double y, double z) {
            pos_ = glm::dvec3(x, y, z);
            return *this;
        }

        PositionIntegrator& set_pos(const glm::dvec3& pos) {
            pos_ = pos;
            return *this;
        }

        PositionIntegrator& set_vel(const glm::dvec3& vel) {
            vel_ = vel;
            return *this;
        }

        PositionIntegrator& reset_acc() {
            acc_ = glm::dvec3(0);
            return *this;
        }

        PositionIntegrator& add_acc(const glm::dvec3& acc) {
            acc_ += acc;
            return *this;
        }

        void integrate(double dt) {
            vel_ += acc_ * dt;
            pos_ += vel_ * dt;
        }

    private:
        glm::dvec3 pos_;
        glm::dvec3 vel_;
        glm::dvec3 acc_;
    };


    class SimpleFixedWing : public Entity {

    public:
        SimpleFixedWing() : quat_(1, 0, 0, 0) {
            pos_.set_pos(-3049328.82, 4049133.27, 3859976.86);
        }

        const glm::dvec3& pos() override { return pos_.pos(); }
        const glm::dvec3& vel() override { return pos_.vel(); }
        const glm::dvec3& acc() override { return pos_.acc(); }

        void update(double dt) {
            const auto anti_gravity_n = glm::normalize(pos_.pos());

            // Adjust roll
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_right());
                quat_ = glm::rotate(quat_, align * dt, this->entt_front());
            }

            // Adjust elevation
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_front());
                quat_ = glm::rotate(quat_, align * dt, this->entt_right());
            }

            // Head forward
            {
                const double dst_speed = 1000000.0;
                const auto align = glm::dot(anti_gravity_n, this->entt_up());
                pos_.reset_acc();
                if (align > 0.999) {
                    pos_.add_acc(this->entt_front() * 1000.0);
                }
            }

            pos_.integrate(dt);
        }

        glm::dvec3 make_eular() const {
            const auto eular = glm::eulerAngles(quat_);
            return glm::dvec3(eular.z, eular.y, eular.x);
        }

        glm::dvec3 entt_front() const {
            return quat_ * CENTER_TO_PRIME_MERIDIAN;
        }
        glm::dvec3 entt_up() const { return quat_ * (-CENTER_TO_NORTH); }
        glm::dvec3 entt_right() const { return quat_ * (CENTER_TO_ASIA); }

    private:
        glm::dquat quat_;         // Quaternion
        PositionIntegrator pos_;  // Geocentric position
        double speed_ = 0;        // Speed in m/s
    };

}  // namespace


namespace {


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

            fixed_wing_.name_ = "Jay 20 fixed wing";
            fixed_wing_.dis_id_ = 1;

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

            disordat::EnttPdu pdu;
            pdu.header_.set_default()
                .set_type(disordat::PduType::entity_state)
                .set_len(sizeof(disordat::EnttPdu));
            pdu.entt_id_.set(::SITE_ID, ::APP_ID, fixed_wing_.dis_id_);
            pdu.force_id_ = 3;
            pdu.num_of_articulation_param_ = 0;
            pdu.entt_type_.set(1, 2, 45, 1, 7, 0, 0);
            pdu.alt_entt_type_.set(1, 1, 225, 1, 1, 1, 0);
            pdu.entt_linear_vel_.set(fixed_wing_.vel());
            pdu.entt_loc_.set(fixed_wing_.pos());
            pdu.entt_orient_.set(fixed_wing_.make_eular());
            pdu.entt_appearance_.clear();
            pdu.dead_reckoning_param_.set_default()
                .set_algorithm(4)
                .set_linear_acc(fixed_wing_.acc());
            pdu.entt_marking_.set_ascii(fixed_wing_.name_);
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
            if (eps_to_ignore_.find(remote_ep_tmp_) != eps_to_ignore_.end())
                return this->start_recv();

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

            if (bytes_transferred < sizeof(disordat::PduHeader)) {
                ss << fmt::format(
                    "Received bytes is less than PDU header size\n"
                );
                return this->start_recv();
            }

            const auto pdu_header = reinterpret_cast<disordat::PduHeader*>(
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

                const auto entt_pdu = reinterpret_cast<disordat::EnttPdu*>(
                    recv_buffer_.data()
                );

                if (entt_pdu->entt_id_.site_id() == ::SITE_ID &&
                    entt_pdu->entt_id_.app_id() == ::APP_ID) {
                    eps_to_ignore_.insert(remote_ep_tmp_);
                    return this->start_recv();
                }

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

                const auto data_pdu = reinterpret_cast<disordat::DataPdu*>(
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
                const auto fixed_data = reinterpret_cast<disordat::FixedDatum*>(
                    recv_buffer_.data() + sizeof(disordat::DataPdu)
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
        std::set<asio::ip::udp::endpoint> eps_to_ignore_;
    };

}  // namespace


int main() {
    spdlog::set_level(spdlog::level::debug);

    asio::io_context io_context;
    ::UdpServer server(io_context);
    std::thread th([&] { io_context.run(); });

    ::UdpRadioTower radio_tower(io_context);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
