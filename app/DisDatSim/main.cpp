#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#define GLM_ENABLE_EXPERIMENTAL
#define SPDLOG_ACTIVE_LEVEL 0
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <asio.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <sung/general/time.hpp>

#include "disordat/pdu_struct.hpp"


namespace {

    int SITE_ID = 1;
    int APP_ID = 3003;

    const glm::dvec3 CENTER_TO_PRIME_MERIDIAN{ 1, 0, 0 };
    const glm::dvec3 CENTER_TO_ASIA{ 0, 1, 0 };
    const glm::dvec3 CENTER_TO_NORTH{ 0, 0, 1 };


    template <typename T>
    class MutsukiList {

    public:
        size_t size() {
            std::lock_guard<std::mutex> lock(mutsuki_);
            return data_.size();
        }

        void push_back(T& value) {
            std::lock_guard<std::mutex> lock(mutsuki_);
            data_.emplace_back(value);
        }

        void push_back(T&& value) {
            std::lock_guard<std::mutex> lock(mutsuki_);
            data_.emplace_back(std::move(value));
        }

        T* try_get(size_t index) {
            std::lock_guard<std::mutex> lock(mutsuki_);
            if (index >= data_.size())
                return nullptr;
            return &data_[index];
        }

        void erase_if(std::function<bool(T&)> pred) {
            std::lock_guard<std::mutex> lock(mutsuki_);

            for (auto it = data_.begin(); it != data_.end();) {
                if (pred(*it)) {
                    it = data_.erase(it);
                } else {
                    ++it;
                }
            }
        }

    private:
        std::vector<T> data_;
        std::mutex mutsuki_;
    };


    class RemoteEntities {

    public:
        void notify(const disordat::EnttPdu& pdu) {
            const auto key = pdu.entt_id_.to_str();
            auto entt = entt_.get_or_create(key);
            entt->update_pos(pdu.entt_loc_.get());

            entt_.erase_if([](const auto& key, auto& entt) {
                return entt.has_elapsed(5.0);
            });
        }

        sung::Optional<glm::dvec3> select_target_pos() {
            auto lock = entt_.lock();
            for (auto& e : entt_) {
                return e.second->pos();
            }

            return sung::nullopt;
        }

    private:
        class Entity {

        public:
            void update_pos(const glm::dvec3& pos) {
                std::lock_guard<std::mutex> lock(mutsuki_);
                pos_ = pos;
                last_update_.check();
            }

            glm::dvec3 pos() {
                std::lock_guard<std::mutex> lock(mutsuki_);
                return pos_;
            }

            bool has_elapsed(double sec) {
                std::lock_guard<std::mutex> lock(mutsuki_);
                return last_update_.has_elapsed(sec);
            }

        private:
            glm::dvec3 pos_;
            sung::MonotonicRealtimeTimer last_update_;
            std::mutex mutsuki_;
        };

        class EntityMap {

        public:
            using map_t =
                std::unordered_map<std::string, std::shared_ptr<Entity>>;

            std::shared_ptr<Entity> find(const std::string& key) {
                std::lock_guard<std::mutex> lock(mutsuki_);

                auto it = data_.find(key);
                if (it == data_.end())
                    return nullptr;
                return it->second;
            }

            std::shared_ptr<Entity> get_or_create(const std::string& key) {
                std::lock_guard<std::mutex> lock(mutsuki_);

                auto it = data_.find(key);
                if (it != data_.end())
                    return it->second;

                auto e = std::make_shared<Entity>();
                data_[key] = e;
                return e;
            }

            void erase_if(std::function<bool(const std::string&, Entity&)> pred
            ) {
                std::lock_guard<std::mutex> lock(mutsuki_);

                for (auto it = data_.begin(); it != data_.end();) {
                    if (!it->second) {
                        it = data_.erase(it);
                    } else if (pred(it->first, *it->second)) {
                        it = data_.erase(it);
                    } else {
                        ++it;
                    }
                }
            }

            bool empty() {
                std::lock_guard<std::mutex> lock(mutsuki_);
                return data_.empty();
            }

            std::lock_guard<std::mutex> lock() {
                return std::lock_guard<std::mutex>(mutsuki_);
            }
            map_t::iterator begin() { return data_.begin(); }
            map_t::iterator end() { return data_.end(); }

        public:
            map_t data_;
            std::mutex mutsuki_;
        };

        EntityMap entt_;
    };


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
            pos_ += vel_ * dt;
            vel_ += acc_ * dt;
        }

    private:
        glm::dvec3 pos_;
        glm::dvec3 vel_;
        glm::dvec3 acc_;
    };


    glm::dvec3 to_euler(const glm::dquat& q) {
        glm::mat4 m = mat4_cast(q);

        float y, p, r;
        glm::extractEulerAngleZYX(m, y, p, r);

        return glm::vec3{ y, p, r };
    }


    class AirplaneIntegrator {

    public:
        AirplaneIntegrator() : quat_(1, 0, 0, 0) {}

        const glm::dquat& quat() const { return quat_; }

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

        glm::dvec3 make_eular() const { return ::to_euler(quat_); }

        void integrate(double dt) {
            this->rot_head(vel_head_ * dt);
            this->rot_elev(vel_elev_ * dt);
            this->rot_roll(vel_roll_ * dt);
        }

        double vel_head_ = 0;
        double vel_elev_ = 0;
        double vel_roll_ = 0;

    private:
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

        glm::dquat quat_;
    };


    class SimpleFixedWing : public Entity {

    public:
        const glm::dvec3& pos() override { return pos_.pos(); }
        const glm::dvec3& vel() override { return pos_.vel(); }
        const glm::dvec3& acc() override { return pos_.acc(); }

        glm::dvec3 make_eular() const { return ori_.make_eular(); }
        glm::dvec3 make_rotational_vel() const {
            return glm::dvec3(ori_.vel_roll_, ori_.vel_elev_, ori_.vel_head_);
        }

        glm::dvec3 entt_front() const { return ori_.entt_front(); }
        glm::dvec3 entt_up() const { return ori_.entt_up(); }
        glm::dvec3 entt_right() const { return ori_.entt_right(); }

        void update(double dt) {
            ori_.integrate(dt);

            pos_.integrate(dt);
            pos_.reset_acc();
            pos_.add_acc(this->entt_front() * speed_.ms());
            pos_.add_acc(pos_.vel() * -1.0);
            pos_.add_acc(glm::normalize(pos_.pos()) * (-9.8));
        }

        void set_pos(const glm::dvec3& v) { pos_.set_pos(v); }

        void set_speed(disordat::Speed speed) { speed_ = speed; }
        void add_speed(disordat::Speed speed) { speed_ = speed_ + speed; }

        void perform_straight_flight(double dt) {
            const auto anti_gravity_n = glm::normalize(pos_.pos());

            // Adjust roll
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_right());
                ori_.vel_roll_ = align;
            }

            // Adjust elevation
            {
                const auto align = glm::dot(anti_gravity_n, this->entt_front());
                ori_.vel_elev_ = align;
            }
        }

        void perform_head_along(double dt, glm::dvec3 direc) {
            direc = glm::normalize(direc);
            const auto dst_front_align = glm::dot(direc, this->entt_front());
            const auto angle_off = sung::acos_safe(dst_front_align);

            const auto dst_right_align = glm::dot(direc, this->entt_right());
            const auto dst_up_align = glm::dot(direc, this->entt_up());

            if (dst_up_align > 0) {
                ori_.vel_roll_ = dst_right_align * 0.4;
                ori_.vel_elev_ = dst_up_align * 0.2;
            } else {
                const auto dst_ailer = sung::signum(dst_right_align);
                ori_.vel_roll_ = dst_ailer * 0.2;
            }
        }

        auto lock() { return std::lock_guard<std::mutex>(mutsuki_); }

    private:
        std::mutex mutsuki_;
        AirplaneIntegrator ori_;  // Quaternion
        PositionIntegrator pos_;  // Geocentric position
        disordat::Speed speed_;
    };


    class Scene {

    public:
        Scene() {
            const glm::dvec3 init_pos(-3049328.82, 4049133.27, 3859976.86);
            auto e = std::make_shared<SimpleFixedWing>();
            e->name_ = "Jay 20 fixed wing";
            e->dis_id_ = 1;
            e->set_pos(init_pos);
            local_entt_.push_back(e);
        }

        void do_frame() {
            const auto dt = timer_.check_get_elapsed();

            for (size_t i = 0; i < local_entt_.size(); ++i) {
                auto entt_ptr = local_entt_.try_get(i);
                if (nullptr == entt_ptr)
                    break;
                auto entt = *entt_ptr;
                auto lock = entt->lock();

                if (auto target_pos = remote_entt_.select_target_pos()) {
                    const auto to_target = *target_pos - entt->pos();
                    entt->perform_head_along(dt, to_target);
                    entt->set_speed(disordat::Speed::from_kts(700));
                } else {
                    SPDLOG_WARN("No target found");
                }
                entt->update(dt);
            }
        }

        ::MutsukiList<std::shared_ptr<SimpleFixedWing>> local_entt_;
        ::RemoteEntities remote_entt_;

    private:
        sung::MonotonicRealtimeTimer timer_;
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
        UdpRadioTower(asio::io_context& io_context, ::Scene& scene)
            : scene_(scene)
            , io_context_(io_context)
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
            constexpr auto PACKETS_PER_SEC = 2.0;
            constexpr auto MS_INTERVAL = 1000.0 / PACKETS_PER_SEC;
            constexpr auto MS_INTERVAL_INT = static_cast<int>(MS_INTERVAL);
            tick_timer_ = asio::steady_timer(
                io_context_, std::chrono::milliseconds(MS_INTERVAL_INT)
            );
            tick_timer_.async_wait(std::bind(&UdpRadioTower::tick, this));
        }

        void tick() {
            for (size_t i = 0; i < scene_.local_entt_.size(); ++i) {
                auto entt_ptr = scene_.local_entt_.try_get(i);
                if (nullptr == entt_ptr)
                    break;
                auto e = *entt_ptr;
                auto lock = e->lock();

                disordat::EnttPdu pdu;
                pdu.header_.set_default()
                    .set_type(disordat::PduType::entity_state)
                    .set_len(sizeof(disordat::EnttPdu));
                pdu.entt_id_.set(::SITE_ID, ::APP_ID, e->dis_id_);
                pdu.force_id_ = 2;
                pdu.num_of_articulation_param_ = 0;
                pdu.entt_type_.set(1, 2, 45, 1, 7, 0, 0);
                pdu.alt_entt_type_.set(1, 1, 225, 1, 1, 1, 0);
                pdu.entt_linear_vel_.set(e->vel());
                pdu.entt_loc_.set(e->pos());
                pdu.entt_orient_.set(e->make_eular());
                pdu.entt_appearance_.clear();
                pdu.dead_reckoning_param_.set_default()
                    .set_algorithm(4)
                    .set_linear_acc(e->acc())
                    .set_angular_vel(e->make_rotational_vel());
                pdu.entt_marking_.set_ascii(e->name_);
                pdu.entt_capabilities_.set(0);
                pdu.padding_.fill(0);
                pdu.header_.set_timestamp();

                socket_.send_to(asio::buffer(&pdu, sizeof(pdu)), endpoint_);
            }

            this->start_tick();
        }

        ::Scene& scene_;
        asio::io_context& io_context_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint endpoint_;
        asio::steady_timer tick_timer_;
    };


    class UdpServer {

    public:
        UdpServer(asio::io_context& io_context, ::Scene& scene)
            : scene_(scene)
            , ep_(asio::ip::udp::v4(), 3000)
            , socket_(io_context, ep_) {
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

            SPDLOG_INFO(
                "Timestamp: {}", pdu_header->timestamp_.make_readable()
            );

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
                scene_.remote_entt_.notify(*entt_pdu);
            } else if (pdu_header->pdu_type_ == 20) {
                const auto data_pdu = reinterpret_cast<disordat::DataPdu*>(
                    recv_buffer_.data()
                );
                ss << data_pdu->make_readable();
            }

            // SPDLOG_INFO(ss.str().substr(0, ss.str().size() - 1));
            this->start_recv();
        }

        ::Scene& scene_;
        asio::ip::udp::endpoint ep_;
        asio::ip::udp::socket socket_;
        asio::ip::udp::endpoint remote_ep_tmp_;
        std::array<char, 1024 * 8> recv_buffer_;
        std::set<asio::ip::udp::endpoint> eps_to_ignore_;
    };

}  // namespace


int main() {
    spdlog::set_level(spdlog::level::debug);

    ::Scene scene;

    asio::io_context io_context;
    ::UdpServer server(io_context, scene);
    ::UdpRadioTower radio_tower(io_context, scene);
    std::thread th([&] { io_context.run(); });

    while (true) {
        scene.do_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 30));
    }

    return 0;
}
