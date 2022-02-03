#ifndef PIN_DEF_H
#define PIN_DEF_H

//###################################################
//              BOARD PIN DEFINITION
//###################################################

#define SD_LED PA_0
#define KILL_LED PA_1
#define SD_CS PA_2
#define RESET_LED PA_3
#define A1 PA_4
#define A0 PA_5
#define LATCH_DEMUX PA_6
#define A2 PA_7
#define I2C_SCL PA_8

#define SEND_SD_RS PB_0
#define SEND_TO_SD PB_1
#define PWM_STOP PB_2
#define ALERT8 PB_4
#define KILL_3V3 PB_5
#define I2C2_SCL PB_6
#define I2C2_SDA PB_7
#define FAN2 PB_8
#define KILL_ENABLE PB_9
#define MTR1 PB_10
#define STATUS1 PB_12
#define ALERT1 PB_13
#define MTR6 PB_14
#define STATUS6 PB_15

#define A3 PC_4
#define E_DEMUX PC_5
#define PWM7 PC_6
#define PWM3 PC_7
#define PWM8 PC_8
#define PWM4_I2C_SDA PC_9
#define MTR3 PC_10
#define STATUS3 PC_11
#define ALERT3 PC_12

#define MTR7 PD_0
#define STATUS7 PD_1
#define ALERT7 PD_2
#define MTR4 PD_3
#define STATUS4 PD_4
#define ALERT4 PD_5
#define MTR8 PD_6
#define STATUS8 PD_7
#define ALERT6 PD_8
#define MTR2 PD_9
#define STATUS2 PD_10
#define ALERT2 PD_11
#define PWM5 PD_12
#define PWM1 PD_13
#define PWM6 PD_14
#define PWM2 PD_15

#define RED_TRISTATE PE_1
#define SCLK PE_2
#define GREEN_TRISTATE PE_3
#define YELLOW_TRISTATE PE_4
#define MISO_PWM4 PE_5
#define MOSI PE_6
#define FAN1 PE_7
#define ALERT9 PE_8
#define ALERT10 PE_9
#define MTR5 PE_10
#define STATUS5 PE_11
#define SCLK_SD PE_12
#define MISO_SD PE_13
#define MOSI_SD PE_14
#define ALERT5 PE_15

//###################################################
//              RS485 PIN DEFINITION
//###################################################

#define RS485_RX_PIN PA_10
#define RS485_TX_PIN PA_9

#define RS485_TE_PIN PA_12
#define RS485_DE_PIN PA_11
#define RS485_RE_PIN PC_0

#endif