/*
 * telemetry.h
 *
 *  Created on: May 12, 2021
 *      Author: elvin
 */

#ifndef INC_DRIVERS_TELEMETRY_H_
#define INC_DRIVERS_TELEMETRY_H_

#define packet_state_ranging
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

typedef struct __attribute__((packed)) {
    uint8_t packet_state_bits; // <[]>
    uint8_t pin_states_servo;
    uint8_t systick[3]; // equivalent to 24 bit unsigned integer
    uint8_t vbat_MSB;
    uint8_t padding_vbat_LSB;
    int8_t orientation_quat[4]; // [w,x,y,z]
    int16_t acc[3];
    int16_t gyro[3];
    uint16_t baro;
    uint8_t temp; // temperature in celsius
    uint16_t altitude;
    uint8_t vertical_velocity;
    uint16_t debug_ranging;
} TLM_encoded;

typedef struct {
    uint8_t packet_type;
    uint8_t flight_state;
    uint8_t is_playing_music;
    uint8_t is_data_logging;
    uint8_t pin_states;
    uint8_t servo_state;
    uint32_t systick;
    float vbat;
    float orientation_quat[4]; // [w,x,y,z] MUST be normalized
    int16_t acc[3];
    int16_t gyro[3];
    float baro;
    float temp; // temperature in celsius
    float altitude;
    float vertical_velocity;
    uint16_t debug;
    float ranging;

} TLM_decoded;

void encode_TLM(TLM_decoded *dec, TLM_encoded *enc);
void decode_TLM(TLM_encoded *enc, TLM_decoded *dec);
void test_TLM();
#endif /* INC_DRIVERS_TELEMETRY_H_ */
