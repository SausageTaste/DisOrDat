#include "disordat/pdu_struct.hpp"

#include <bitset>
#include <sstream>

#include <spdlog/spdlog.h>
#include <sung/general/time.hpp>


namespace {

    template <uint64_t base, uint64_t exp>
    constexpr uint64_t int_pow() {
        uint64_t result = 1;
        for (size_t i = 0; i < exp; ++i) {
            result *= base;
        }
        return result;
    }

    constexpr auto TIMESTAMP_UNITS = ::int_pow<2, 31>() - 1;
    constexpr auto TIMESTAMP_FACTOR = TIMESTAMP_UNITS / (60.0 * 60.0);

}  // namespace


namespace disordat {

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

    std::string to_str(const Vec3& v) {
        return fmt::format("({:.2f}, {:.2f}, {:.2f})", v.x, v.y, v.z);
    }

}  // namespace disordat


// Timestamp
namespace disordat {

    double Timestamp::get_sec() const {
        const auto ptr = reinterpret_cast<const uint32_t*>(value_.data());
        auto x = sung::flip_byte_order(*ptr);
        x >>= 1;
        return x / TIMESTAMP_FACTOR;
    }

    uint8_t Timestamp::get_mode() const {
        const auto ptr = reinterpret_cast<const uint32_t*>(value_.data());
        auto x = sung::flip_byte_order(*ptr);
        return x & 0x01;
    }

    std::string Timestamp::make_readable() const {
        return fmt::format("{} ({})", this->get_sec(), this->get_mode());
    }

    void Timestamp::set_now() {
        const auto now = sung::get_time_unix();
        auto ptr = reinterpret_cast<uint32_t*>(value_.data());
        *ptr = static_cast<uint32_t>(now * TIMESTAMP_FACTOR);
        *ptr <<= 1;
        *ptr &= 0xFFFFFFFE;
        *ptr = sung::flip_byte_order(*ptr);
    }

}  // namespace disordat


// PduHeader
namespace disordat {

    PduType PduHeader::pdu_type() const {
        return static_cast<PduType>(pdu_type_);
    }

    const char* PduHeader::pdu_type_str() const {
        return to_str(this->pdu_type());
    }

    PduHeader& PduHeader::set_default() {
        version_ = 7;
        exercise_id_ = 1;
        pdu_type_ = 0;
        protocol_family_ = 5;
        timestamp_.set_now();
        length_ = 0;
        padding_ = 0;
        return *this;
    }

    PduHeader& PduHeader::set_type(PduType pdu_type) {
        pdu_type_ = static_cast<uint8_t>(pdu_type);
        return *this;
    }

    PduHeader& PduHeader::set_timestamp() {
        timestamp_.set_now();
        return *this;
    }

    PduHeader& PduHeader::set_len(uint16_t len) {
        length_ = len;
        return *this;
    }

}  // namespace disordat


// EnttId
namespace disordat {

    uint16_t EnttId::site_id() const { return sim_addr_.site_id_.get(); }
    uint16_t EnttId::app_id() const { return sim_addr_.app_id_.get(); }
    uint16_t EnttId::entt_id() const { return entt_id_.get(); }

    std::string EnttId::to_str() const {
        return fmt::format(
            "{}:{}:{}", this->site_id(), this->app_id(), this->entt_id()
        );
    }

    EnttId& EnttId::set(uint16_t site_id, uint16_t app_id, uint16_t entt_id) {
        sim_addr_.site_id_.set(site_id);
        sim_addr_.app_id_.set(app_id);
        entt_id_.set(entt_id);
        return *this;
    }

}  // namespace disordat


// DeadReckoningParam
namespace disordat {

    DeadReckoningParam& DeadReckoningParam::set_default() {
        algorithm_ = 1;
        other_params_.fill(0);
        linear_acc_.set(0, 0, 0);
        angular_vel_.set(0, 0, 0);
        return *this;
    }

    DeadReckoningParam& DeadReckoningParam::set_algorithm(uint8_t x) {
        algorithm_ = x;
        return *this;
    }

    DeadReckoningParam& DeadReckoningParam::set_linear_acc(const Vec3& acc) {
        linear_acc_.set(acc);
        return *this;
    }

    DeadReckoningParam& DeadReckoningParam::set_angular_vel(const Vec3& vel) {
        angular_vel_.set(vel);
        return *this;
    }

}  // namespace disordat


// EnttMarking
namespace disordat {

    EnttMarking& EnttMarking::clear() {
        charset_ = 0;
        str_.fill(0);
        return *this;
    }

    EnttMarking& EnttMarking::set_ascii(const std::string& str) {
        charset_ = 1;
        str_.fill(0);

        const auto len = std::min(str.size(), str_.size());
        for (size_t i = 0; i < len; ++i) {
            str_[i] = str[i];
        }

        if (len < str.size()) {
            str_[8] = '.';
            str_[9] = '.';
            str_[10] = '.';
        }

        return *this;
    }

}  // namespace disordat


// DataPdu
namespace disordat {

    std::string DataPdu::make_readable() const {
        std::stringstream ss;
        ss << fmt::format("Data PDU\n");

        ss << fmt::format(
            "  originating_entt={}\n", originating_entt_.to_str()
        );

        ss << fmt::format("  receiving_entt={}\n", receiving_entt_.to_str());

        ss << fmt::format(
            "  request_id={}, padding={}, num_of_fixed_datum={}, "
            "num_of_variable_datum={}\n",
            request_id_.get(),
            padding_.get(),
            num_of_fixed_datum_.get(),
            num_of_variable_datum_.get()
        );

        const auto fixed_data_num = num_of_fixed_datum_.get();
        const auto fixed_data = reinterpret_cast<const disordat::FixedDatum*>(
            reinterpret_cast<const uint8_t*>(this) + sizeof(disordat::DataPdu)
        );
        ss << fmt::format("Fixed data ({})\n", fixed_data_num);
        for (size_t i = 0; i < fixed_data_num; ++i) {
            ss << fmt::format(
                "  id={}, value={}\n",
                fixed_data[i].datum_id_.get(),
                fixed_data[i].datum_value_.get()
            );
        }

        return ss.str();
    }

}  // namespace disordat


// EnttPdu
namespace disordat {

    std::string EnttPdu::make_readable() const {
        std::stringstream ss;
        ss << fmt::format("Entity State PDU\n");

        ss << fmt::format(
            "PDU Header: version={}, exercise_id={}, pdu_type={}, "
            "protocol_family={}, timestamp={}, length={}\n",
            header_.version_,
            header_.exercise_id_,
            header_.pdu_type_str(),
            header_.protocol_family_,
            header_.timestamp_.make_readable(),
            header_.length_.get()
        );

        ss << fmt::format(
            "  entt_id={}, force_id={}, num_of_articulation_param={}\n",
            entt_id_.to_str(),
            force_id_,
            num_of_articulation_param_
        );

        ss << fmt::format(
            "  kind={}, domain={}, country={}, category={}, subcategory={}, "
            "specific={}, extra={}\n",
            entt_type_.kind_,
            entt_type_.domain_,
            entt_type_.country_.get(),
            entt_type_.category_,
            entt_type_.subcategory_,
            entt_type_.specific_,
            entt_type_.extra_
        );

        ss << fmt::format(
            "  alt_kind={}, alt_domain={}, alt_country={}, alt_category={}, "
            "alt_subcategory={}, alt_specific={}, alt_extra={}\n",
            alt_entt_type_.kind_,
            alt_entt_type_.domain_,
            alt_entt_type_.country_.get(),
            alt_entt_type_.category_,
            alt_entt_type_.subcategory_,
            alt_entt_type_.specific_,
            alt_entt_type_.extra_
        );

        ss << fmt::format(
            "  linear_vel={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
            entt_linear_vel_.x_.get(),
            entt_linear_vel_.y_.get(),
            entt_linear_vel_.z_.get()
        );

        ss << fmt::format(
            "  loc={{x={:.2f}, y={:.2f}, z={:.2f}}}\n",
            entt_loc_.x_.get(),
            entt_loc_.y_.get(),
            entt_loc_.z_.get()
        );

        ss << fmt::format(
            "  orient={{psi={:.2f}, theta={:.2f}, phi={:.2f}}}\n",
            entt_orient_.psi_.get(),
            entt_orient_.theta_.get(),
            entt_orient_.phi_.get()
        );

        return ss.str();
    }

}  // namespace disordat
