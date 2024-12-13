#pragma once

#include <glm/vec3.hpp>
#include <sung/general/angle.hpp>
#include <sung/general/bytes.hpp>
#include <sung/general/units.hpp>


namespace disordat {

    using Vec3 = glm::dvec3;
    using Angle = sung::TAngle<double>;
    using Speed = sung::TSpeed<double>;


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

    const char* to_str(PduType pdu_type);
    std::string to_str(const Vec3& v);


    class PduHeader {

    public:
        PduType pdu_type() const;
        const char* pdu_type_str() const;

        PduHeader& set_default();
        PduHeader& set_type(PduType pdu_type);
        PduHeader& set_timestamp();
        PduHeader& set_len(uint16_t len);

        uint8_t version_;
        uint8_t exercise_id_;
        uint8_t pdu_type_;
        uint8_t protocol_family_;
        sung::BEValue<uint32_t> timestamp_;
        sung::BEValue<uint16_t> length_;
        sung::BEValue<uint16_t> padding_;
    };
    static_assert(8 * sizeof(PduHeader) == 96, "");


    struct SimAdress {
        sung::BEValue<uint16_t> site_id_;
        sung::BEValue<uint16_t> app_id_;
    };


    class EnttId {

    public:
        uint16_t site_id() const;
        uint16_t app_id() const;
        uint16_t entt_id() const;

        std::string to_str() const;

        EnttId& set(uint16_t site_id, uint16_t app_id, uint16_t entt_id);

    private:
        SimAdress sim_addr_;
        sung::BEValue<uint16_t> entt_id_;
    };
    static_assert(8 * sizeof(EnttId) == 48, "");


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
    static_assert(8 * sizeof(EnttType) == 64, "");


    struct LinearVelVector {
        Vec3 get() const { return Vec3(x_.get(), y_.get(), z_.get()); }

        LinearVelVector& set(float x, float y, float z) {
            x_.set(x);
            y_.set(y);
            z_.set(z);
            return *this;
        }

        LinearVelVector& set(const Vec3& v) {
            x_.set(v.x);
            y_.set(v.y);
            z_.set(v.z);
            return *this;
        }

        sung::BEValue<float> x_;
        sung::BEValue<float> y_;
        sung::BEValue<float> z_;
    };
    static_assert(8 * sizeof(LinearVelVector) == 96, "");


    struct WorldCoord {
        Vec3 get() const { return Vec3(x_.get(), y_.get(), z_.get()); }

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
    static_assert(8 * sizeof(WorldCoord) == 192, "");


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
    static_assert(8 * sizeof(EulerAngles) == 96, "");


    struct EnttAppearance {
        EnttAppearance& clear() {
            general_apperaance_.set(0);
            specific_appearance_varient_.set(0);
            return *this;
        }

        sung::BEValue<uint16_t> general_apperaance_;
        sung::BEValue<uint16_t> specific_appearance_varient_;
    };
    static_assert(8 * sizeof(EnttAppearance) == 32, "");


    class DeadReckoningParam {

    public:
        DeadReckoningParam& set_default();
        DeadReckoningParam& set_algorithm(uint8_t x);
        DeadReckoningParam& set_linear_acc(const Vec3& acc);
        DeadReckoningParam& set_angular_vel(const Vec3& vel);

    private:
        uint8_t algorithm_;
        std::array<uint8_t, 15> other_params_;
        LinearVelVector linear_acc_;
        LinearVelVector angular_vel_;
    };
    static_assert(8 * sizeof(DeadReckoningParam) == 320, "");


    struct EnttMarking {
        EnttMarking& clear();
        EnttMarking& set_ascii(const std::string& str);

        uint8_t charset_;
        std::array<uint8_t, 11> str_;
    };
    static_assert(8 * sizeof(EnttMarking) == 96, "");


    struct DataPdu {
        std::string make_readable() const;

        PduHeader header_;
        EnttId originating_entt_;
        EnttId receiving_entt_;
        sung::BEValue<uint32_t> request_id_;
        sung::BEValue<uint32_t> padding_;
        sung::BEValue<uint32_t> num_of_fixed_datum_;
        sung::BEValue<uint32_t> num_of_variable_datum_;
    };


    struct EnttPdu {
        std::string make_readable() const;

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
    static_assert(8 * sizeof(EnttPdu) == 1280, "");

}  // namespace disordat
