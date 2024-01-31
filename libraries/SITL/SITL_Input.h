#pragma once

#include <stdint.h>

/*
  structure passed in giving servo positions as PWM values in
  microseconds
*/
static constexpr uint8_t MAX_VOTE_PINS = 5;
struct sitl_input {
    uint16_t servos[16];
    struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
    } wind;
    uint8_t my_vote_pin;
    uint8_t vote_pin[MAX_VOTE_PINS];
};

