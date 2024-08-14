#pragma once

#include <stdint.h>

#include "AP_GPS_UBLOX_COMMON_def.h"

#include <AP_Param/AP_Param.h>

#define CPU_LOAD_EXPECTED_DEFAULT 85

struct AP_GPS_UBLOX_ILM_CONFIG {
    
    AP_GPS_UBLOX_ILM_CONFIG();

    AP_Float hacc_min;
    AP_Float vacc_min;
    AP_Float hdop_max;
    AP_Float hdop_min;
    AP_Int32 satellites_max;
    AP_Int32 satellites_min;
    AP_Int32 clock_acc_min;
    AP_Int8  target_cn0;
    
    AP_Int16  min_ilm_score;

    static const struct AP_Param::GroupInfo var_info[];
};

class AP_GPS_UBLOX_ILM
{
public:
   explicit AP_GPS_UBLOX_ILM(const AP_GPS_UBLOX_ILM_CONFIG &config);

   int16_t get_score() const;

   int8_t get_hacc_score() const;
   int8_t get_vacc_score() const;
   int8_t get_antenna_score() const;
   int8_t get_jamming_score() const;
   int8_t get_spoofing_score() const;
   int8_t get_yaw_score() const;
   int8_t get_sats_score() const;
   int8_t get_cpu_score() const;
   int8_t get_cn0_score() const;
   int8_t get_dop_score() const;
   int8_t get_fix_score() const;
   int8_t get_corrections_score() const;
   int8_t get_clock_score() const;


   void update_antenna_status(ubx_ant_status &ant_status);
   
   void update_cpu_load(uint8_t cpu_load_percent);
   
   void update_jamming_status(ubx_jam_state jamming_status);
   void update_spoofing_status(ubx_spoof_state spoofing_status);
   
   void update_has_yaw(bool has_yaw);
   
   void update_fix_status(ubx_nav_fix_type fix_status);
   void update_corrections_status(ubx_nav_fix_flags corrections_status);

   void update_hacc(float hacc);
   void update_vacc(float vacc);

   void update_hdop(uint16_t hdop);

   void update_number_of_tracked_satellites(uint8_t sats);

   void update_clock_accuracy(uint32_t clock_accuracy);

   void update_cn0(uint8_t cn0);

private:

    enum ILM_Weight : int8_t {
        ILM_SCORE_AWFUL = -100,
        ILM_SCORE_ERROR = -50,
        ILM_SCORE_POOR = -10,
        ILM_SCORE_BAD = 0,
        ILM_SCORE_DEGRADED = 5,
        ILM_SCORE_MEH = 10,
        ILM_SCORE_OK = 15,
        ILM_SCORE_EMPHASIS = 20,
    };

    // In line monitor scores
    int8_t hacc_score;
    int8_t vacc_score;
    int8_t antenna_score;
    int8_t jamming_score;
    int8_t spoofing_score;
    int8_t yaw_score;
    int8_t sats_score;
    int8_t cpu_score;
    int8_t cn0_score;
    int8_t dop_score;
    int8_t fix_score;
    int8_t corrections_score;
    int8_t clock_score;

    static constexpr uint8_t _cpu_load_expected{CPU_LOAD_EXPECTED_DEFAULT};

    const AP_GPS_UBLOX_ILM_CONFIG &_config;
};