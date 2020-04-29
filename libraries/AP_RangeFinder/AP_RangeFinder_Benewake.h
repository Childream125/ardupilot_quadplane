#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend_Serial
{

public:

<<<<<<< HEAD
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;
=======
    enum benewake_model_type {
        BENEWAKE_TF02 = 0,
        BENEWAKE_TFmini = 1,
        BENEWAKE_TF03 = 2,
    };

    // constructor
    AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            uint8_t serial_instance,
                            benewake_model_type model);

    // static detection function
    static bool detect(uint8_t serial_instance);

    // update state
    void update(void) override;
>>>>>>> myquadplane

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    virtual float model_dist_max_cm() const = 0;
    virtual bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_cm
    bool get_reading(uint16_t &reading_cm) override;

    uint8_t linebuf[10];
    uint8_t linebuf_len;
};
