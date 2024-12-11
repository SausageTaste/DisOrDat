#include <chrono>
#include <iostream>
#include <mutex>
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


    class RemoteEntities {

    public:
        void notify(const disordat::EnttPdu& pdu) {
            std::lock_guard<std::mutex> lock(mutsuki_);

            const auto key = pdu.entt_id_.to_str();
            auto it = entt_.find(key);
            if (it == entt_.end()) {
                it = entt_.emplace(key, Entity{}).first;
            }
            auto& entt = it->second;
            entt.pos_ = pdu.entt_loc_.get();
        }

        sung::Optional<glm::dvec3> select_target_pos() {
            std::lock_guard<std::mutex> lock(mutsuki_);

            if (entt_.empty())
                return sung::nullopt;

            const auto it = entt_.begin();
            return it->second.pos_;
        }

    private:
        struct Entity {
            glm::dvec3 pos_;
        };

        std::unordered_map<std::string, Entity> entt_;
        std::mutex mutsuki_;
    } remote_entt_;


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
            this->rot_roll(glm::radians(45.0));
        }

        const glm::dvec3& pos() override { return pos_.pos(); }
        const glm::dvec3& vel() override { return pos_.vel(); }
        const glm::dvec3& acc() override { return pos_.acc(); }

        glm::dvec3 make_eular() const {
            const auto eular = glm::eulerAngles(quat_);
            return glm::dvec3(eular.z, eular.y, eular.x);
        }

        glm::dvec3 entt_front() const {
            return glm::mat4_cast(quat_) *
                   glm::dvec4(CENTER_TO_PRIME_MERIDIAN, 0);
        }
        glm::dvec3 entt_up() const {
            return glm::mat4_cast(quat_) * glm::dvec4(-CENTER_TO_NORTH, 0);
        }
        glm::dvec3 entt_right() const {
            return glm::normalize(
                glm::mat4_cast(quat_) * glm::dvec4(CENTER_TO_ASIA, 0)
            );
        }

        void update(double dt) {
            pos_.reset_acc();
            pos_.add_acc(this->entt_front() * 500.0);
            pos_.add_acc(pos_.vel() * -1.0);
            pos_.integrate(dt);
        }

        void rot_head(double angle) {
            const glm::dquat iq{ 1, 0, 0, 0 };
            const auto rot = glm::rotate(iq, angle, this->entt_up());
            quat_ = glm::normalize(rot * quat_);
        }
        void rot_roll(double angle) {
            const glm::dquat iq{ 1, 0, 0, 0 };
            const auto rot = glm::rotate(iq, angle, this->entt_front());
            quat_ = glm::normalize(rot * quat_);
        }
        void rot_elev(double angle) {
            const glm::dquat iq{ 1, 0, 0, 0 };
            const auto rot = glm::rotate(iq, angle, this->entt_right());
            quat_ = glm::normalize(rot * quat_);
        }

        void perform_straight_flight(double dt) {
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
        }

        void perform_head_along(double dt, glm::dvec3 direc) {
            direc = glm::normalize(direc);
            const auto dst_front_align = glm::dot(direc, this->entt_front());
            const auto angle_off = sung::acos_safe(dst_front_align);

            const auto dst_right_align = glm::dot(direc, this->entt_right());
            const auto dst_up_align = glm::dot(direc, this->entt_up());

            if (dst_up_align > 0) {
                this->rot_roll(dt * dst_right_align);
                this->rot_elev(dt * dst_up_align);
            } else {
                const auto dst_ailer = sung::signum(dst_right_align);
                this->rot_roll(dt * dst_ailer);
            }
        }

        void perform_test(double dt) {
            this->rot_elev(dt);
            SPDLOG_DEBUG(
                "Right={}, quat=({:.2f}, {:.2f}, {:.2f}, {:.2f})",
                disordat::to_str(this->entt_right()),
                quat_.w,
                quat_.x,
                quat_.y,
                quat_.z
            );
        }

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

    std::string to_str(asio::ip::udp::endpoint ep) {
        return fmt::format("{}:{}", ep.address().to_string(), ep.port());
    }

    asio::ip::address_v4 get_broadcast_ip() {
        return asio::ip::address_v4::from_string("127.255.255.255");
    }


    class UdpRadioTower {

    public:
        UdpRadioTower(asio::io_context& io_context)
            : io_context_(io_context)
            , socket_(io_context)
            , endpoint_(::get_broadcast_ip(), 3000)
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

            if (auto target_pos = remote_entt_.select_target_pos()) {
                const auto to_target = *target_pos - fixed_wing_.pos();
                fixed_wing_.perform_head_along(dt, to_target);
                // fixed_wing_.perform_test(dt);
            }
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
            // SPDLOG_INFO(
            //     "Sent to {}: len={}, type={}",
            //     ::to_str(endpoint_),
            //     pdu.header_.length_.get(),
            //     pdu.header_.pdu_type_str()
            // );

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
            : ep_(asio::ip::udp::v4(), 3000), socket_(io_context, ep_) {
            this->start_recv();
        }

    private:
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
                return this->start_recv();
            }

            if (bytes_transferred < sizeof(disordat::PduHeader)) {
                SPDLOG_WARN("Received data is smaller than PDU header size");
                return this->start_recv();
            }

            const auto pdu_header = reinterpret_cast<disordat::PduHeader*>(
                recv_buffer_.data()
            );

            // SPDLOG_INFO(
            //     "Received from {}: size={}, type={}",
            //     ::to_str(remote_ep_tmp_),
            //     bytes_transferred,
            //     pdu_header->pdu_type_str()
            // );

            std::stringstream ss;
            ss << fmt::format("\nBytes ({})\n", bytes_transferred);
            //::print_bytes(recv_buffer_.data(), bytes_transferred, 16, 2);

            if (pdu_header->pdu_type_ == 1) {
                const auto entt_pdu = reinterpret_cast<disordat::EnttPdu*>(
                    recv_buffer_.data()
                );

                if (entt_pdu->entt_id_.site_id() == ::SITE_ID &&
                    entt_pdu->entt_id_.app_id() == ::APP_ID) {
                    eps_to_ignore_.insert(remote_ep_tmp_);
                    return this->start_recv();
                }

                ss << entt_pdu->make_readable();
                remote_entt_.notify(*entt_pdu);
            } else if (pdu_header->pdu_type_ == 20) {
                const auto data_pdu = reinterpret_cast<disordat::DataPdu*>(
                    recv_buffer_.data()
                );
                ss << data_pdu->make_readable();
            }

            // SPDLOG_INFO(ss.str().substr(0, ss.str().size() - 1));
            this->start_recv();
        }

        asio::ip::udp::endpoint ep_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint remote_ep_tmp_;
        std::array<char, 1024 * 8> recv_buffer_;
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
