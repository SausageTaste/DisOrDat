#include "disordat/pdu_struct.hpp"

#include <spdlog/spdlog.h>
#include <sung/general/time.hpp>


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
        timestamp_.set(sung::get_time_unix());
        length_ = 0;
        padding_ = 0;
        return *this;
    }

    PduHeader& PduHeader::set_type(PduType pdu_type) {
        pdu_type_ = static_cast<uint8_t>(pdu_type);
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
