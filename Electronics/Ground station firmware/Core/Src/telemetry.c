#include "main.h"
#include "telemetry.h"
#include "math.h"


void encode_TLM(TLM_decoded *dec, TLM_encoded *enc) {
    enc->packet_state_bits = (dec->packet_type & (1<<0)) | ((0b00011111 & dec->flight_state) << 1) | ((1 & dec->is_playing_music) << 6) | ((1&dec->is_data_logging) <<7);

    enc->pin_states_servo = (dec->pin_states & 0b00111111) | ((dec->servo_state & 0b00000011) << 6);

    enc->systick[0] = 0xff & (dec->systick >> 16);
    enc->systick[1] = 0xff & (dec->systick >> 8);
    enc->systick[2] = 0xff & dec->systick;

    uint16_t vbat_10mV = (uint16_t) round(dec->vbat * 100); // 0.01 V/lsb

    enc->vbat_MSB = (uint8_t) ((vbat_10mV >> 2) & 0xFF);
    enc->padding_vbat_LSB = (uint8_t) (vbat_10mV & 0b00000011);

    enc->orientation_quat[0] = (int8_t) round(dec->orientation_quat[0]*127);
    enc->orientation_quat[1] = (int8_t) round(dec->orientation_quat[1]*127);
    enc->orientation_quat[2] = (int8_t) round(dec->orientation_quat[2]*127);
    enc->orientation_quat[3] = (int8_t) round(dec->orientation_quat[3]*127);

    for (int i = 0; i < 3; i++) {
        enc->acc[i] = dec->acc[i];
        enc->gyro[i] = dec->gyro[i];
    }

    enc->baro = (uint16_t) round(dec->baro - 50000); // 50000 Pa as zero point. Max ISA height: 5.5 km, max pressure: 115536 Pa

    enc->temp = (uint8_t) round(dec->temp * 4);

    enc->altitude = (uint16_t) round((dec->altitude+48) * 16); // 0.0625m/LSB, -48 m as reference and max 2000 m.

    enc->vertical_velocity = (uint8_t) MIN(MAX(round(dec->vertical_velocity + 80),0),255); // 0-255 maps to -80-175 m/s.

    if (dec->packet_type == 1) {
        enc->debug_ranging = (uint16_t) round(dec->ranging * 4);
    }
    else {
        enc->debug_ranging = dec->debug;
    }
}

void decode_TLM(TLM_encoded *enc, TLM_decoded *dec) {
    dec->packet_type = enc->packet_state_bits & 1;
    dec->flight_state = (enc->packet_state_bits >> 1) & 0b00011111;
    dec->is_playing_music = (enc->packet_state_bits >> 6) & 1;
    dec->is_data_logging = (enc->packet_state_bits >> 7) & 1;

    dec->pin_states = (enc->pin_states_servo & 0b00111111);
    dec->servo_state = (enc->pin_states_servo >> 6) & 0b00000011;

    dec->systick = (enc->systick[0] << 16) | (enc->systick[1] << 8) | (enc->systick[2]);

    dec->vbat = ((((uint16_t) enc->vbat_MSB) << 2) | ((uint16_t) enc->padding_vbat_LSB & 0b00000011 ) )*0.01;

    dec->orientation_quat[0] = ((float) enc->orientation_quat[0])/127;
    dec->orientation_quat[1] = ((float) enc->orientation_quat[1])/127;
    dec->orientation_quat[2] = ((float) enc->orientation_quat[2])/127;
    dec->orientation_quat[3] = ((float) enc->orientation_quat[3])/127;

    for (int i = 0; i < 3; i++) {
        dec->acc[i] = enc->acc[i];
        dec->gyro[i] = enc->gyro[i];
    }

    dec->baro = (float) (enc->baro + 50000);

    dec->temp = ((float) enc->temp)/4;

    dec->altitude = ((float) enc->altitude) * 0.0625 - 48;

    dec->vertical_velocity = (float) (enc->vertical_velocity - 80);

    if (dec->packet_type == 1) {
        dec->ranging = ((float) enc->debug_ranging) * 0.25;
    }
    else {
        dec->debug = (uint16_t) enc->debug_ranging;
    }
}

void test_TLM() {

    TLM_decoded dec;
    TLM_decoded dec_after;
    TLM_encoded enc;

    dec.packet_type = 1;
    dec.flight_state = 23;
    dec.is_playing_music = 0;
    dec.is_data_logging = 0;

    dec.pin_states = 0b00011011;
    dec.servo_state = 3;

    dec.vbat = 7.283;

    dec.systick = 1232432;

    dec.orientation_quat[0] = 0.143123;
    dec.acc[2] = -12343;
    dec.gyro[2] = -21;

    dec.baro = 90001.623;
    dec.temp = 63.4;
    dec.vertical_velocity = 180;
    dec.altitude = 1321;

    dec.debug = 1337;
    dec.ranging = 15212;

    encode_TLM(&dec, &enc);
    decode_TLM(&enc, &dec_after);

    printf("%i:%i\n", dec.packet_type, dec_after.packet_type);
    printf("%d:%d\n", dec.flight_state, dec_after.flight_state);
    printf("%d:%d\n", dec.is_playing_music, dec_after.is_playing_music);
    printf("%d:%d\n", dec.is_data_logging, dec_after.is_data_logging);

    printf("%d:%d\n", dec.servo_state, dec_after.servo_state);
    printf("%i:%i\n", dec.systick, dec_after.systick);
    printf("%f:%f\n", dec.vbat, dec_after.vbat);
    printf("%f:%f\n", dec.orientation_quat[0], dec_after.orientation_quat[0]);
    printf("%d:%d\n", dec.acc[2], dec_after.acc[2]);
    printf("%d:%d\n", dec.gyro[2], dec_after.gyro[2]);
    printf("%f:%f\n", dec.baro, dec_after.baro);
    printf("%f:%f\n", dec.vertical_velocity, dec_after.vertical_velocity);
    printf("%f:%f\n", dec.temp, dec_after.temp);
    printf("%f:%f\n", dec.altitude, dec_after.altitude);
    printf("%f:%f\n", dec.ranging, dec_after.ranging);
    printf("%d:%d\n", dec.debug, dec_after.debug);

    return 0;
}
