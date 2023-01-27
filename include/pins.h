
#ifndef PICO_SCORBOT_PINS_H
#define PICO_SCORBOT_PINS_H
 
#define LED_PIN 25

/*################ GROUP 1 - MASTER ################*/
#ifdef MASTER

#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

/*-------------- MOTOR PINS --------------*/
#define M0_ENA_PIN 10
#define M0_ENB_PIN 11
#define M1_ENA_PIN 12
#define M1_ENB_PIN 13
#define M2_ENA_PIN 14
#define M2_ENB_PIN 15

/*------------- ENCODER PINS -------------*/
#define M0_ENC_A_PIN 19 // 21
#define M0_ENC_B_PIN 18 // 20
#define M1_ENC_A_PIN 21 // 19
#define M1_ENC_B_PIN 20 // 18
#define M2_ENC_A_PIN 17 // 17
#define M2_ENC_B_PIN 16 // 16

/*------------- SW_HOME PINS -------------*/
#define M0_HOME_SW 7
#define M1_HOME_SW 8
#define M2_HOME_SW 9

#define M0_ENC_INVERTED false
#define M1_ENC_INVERTED true
#define M2_ENC_INVERTED true

#endif
/*################ GROUP 2 - SLAVE ################*/
#ifdef SLAVE

#define I2C_PORT i2c1
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27
/*-------------- MOTOR PINS --------------*/
#define M3_ENA_PIN 10
#define M3_ENB_PIN 11
#define M4_ENA_PIN 12
#define M4_ENB_PIN 13
#define M5_ENA_PIN 14
#define M5_ENB_PIN 15

/*------------- ENCODER PINS -------------*/
#define M3_ENC_A_PIN 21
#define M3_ENC_B_PIN 20
#define M4_ENC_A_PIN 19
#define M4_ENC_B_PIN 18
#define M5_ENC_A_PIN 17
#define M5_ENC_B_PIN 16

/*------------- SW_HOME PINS -------------*/
#define M3_HOME_SW 7
#define M4_HOME_SW 8
#define M5_HOME_SW 9

#define M3_ENC_INVERTED true
#define M4_ENC_INVERTED false
#define M5_ENC_INVERTED true

#endif

#endif //PICO_REMOTE_CAR_PINS_H
