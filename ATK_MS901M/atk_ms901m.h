/**
 ****************************************************************************************************
 * @file        atk_ms901m.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS901Mģ����������
 * @license     Copyright (c) 2020-2032, �������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATM_MS901M_H
#define __ATM_MS901M_H
/**
YAW->Z
PITCH->X
ROLL->Y
**/
#include "sys.h"
#include "atk_ms901m_uart.h"
#include "main.h"
/* ATK-MS901M UARTͨѶ֡������󳤶� */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE       28
/* ATK-MS901M�����ϴ�֡ID */
#define ATK_MS901M_FRAME_ID_ATTITUDE        0x01    /* ��̬�� */
#define ATK_MS901M_FRAME_ID_QUAT            0x02    /* ��Ԫ�� */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE       0x03    /* �����ǡ����ٶȼ� */
#define ATK_MS901M_FRAME_ID_MAG             0x04    /* ������ */
#define ATK_MS901M_FRAME_ID_BARO            0x05    /* ��ѹ�� */
#define ATK_MS901M_FRAME_ID_PORT            0x06    /* �˿� */

/* ATK-MS901MӦ��֡ID */
#define ATK_MS901M_FRAME_ID_REG_SAVE        0x00    /* ��  W�����浱ǰ���õ�Flash */
#define ATK_MS901M_FRAME_ID_REG_SENCAL      0x01    /* ��  W�����ô�����У׼ */
#define ATK_MS901M_FRAME_ID_REG_SENSTA      0x02    /* ��R  ����ȡ������У׼״̬ */
#define ATK_MS901M_FRAME_ID_REG_GYROFSR     0x03    /* ��R/W���������������� */
#define ATK_MS901M_FRAME_ID_REG_ACCFSR      0x04    /* ��R/W�����ü��ٶȼ����� */
#define ATK_MS901M_FRAME_ID_REG_GYROBW      0x05    /* ��R/W�����������Ǵ��� */
#define ATK_MS901M_FRAME_ID_REG_ACCBW       0x06    /* ��R/W�����ü��ٶȼƴ��� */
#define ATK_MS901M_FRAME_ID_REG_BAUD        0x07    /* ��R/W������UARTͨѶ������ */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET   0x08    /* ��R/W�����ûش����� */
#define ATK_MS901M_FRAME_ID_REG_RETURNSET2  0x09    /* ��R/W�����ûش�����2�������� */
#define ATK_MS901M_FRAME_ID_REG_RETURNRATE  0x0A    /* ��R/W�����ûش����� */
#define ATK_MS901M_FRAME_ID_REG_ALG         0x0B    /* ��R/W�������㷨 */
#define ATK_MS901M_FRAME_ID_REG_ASM         0x0C    /* ��R/W�����ð�װ���� */
#define ATK_MS901M_FRAME_ID_REG_GAUCAL      0x0D    /* ��R/W��������������У׼���� */
#define ATK_MS901M_FRAME_ID_REG_BAUCAL      0x0E    /* ��R/W��������ѹ����У׼���� */
#define ATK_MS901M_FRAME_ID_REG_LEDOFF      0x0F    /* ��R/W������LED���� */
#define ATK_MS901M_FRAME_ID_REG_D0MODE      0x10    /* ��R/W�����ö˿�D0ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D1MODE      0x11    /* ��R/W�����ö˿�D1ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D2MODE      0x12    /* ��R/W�����ö˿�D2ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D3MODE      0x13    /* ��R/W�����ö˿�D3ģʽ */
#define ATK_MS901M_FRAME_ID_REG_D1PULSE     0x16    /* ��R/W�����ö˿�D1 PWM�ߵ�ƽ���� */
#define ATK_MS901M_FRAME_ID_REG_D3PULSE     0x1A    /* ��R/W�����ö˿�D3 PWM�ߵ�ƽ���� */
#define ATK_MS901M_FRAME_ID_REG_D1PERIOD    0x1F    /* ��R/W�����ö˿�D1 PWM���� */
#define ATK_MS901M_FRAME_ID_REG_D3PERIOD    0x23    /* ��R/W�����ö˿�D3 PWM���� */
#define ATK_MS901M_FRAME_ID_REG_RESET       0x7F    /* ��  W���ָ�Ĭ������ */

/* ATK-MS901M֡���� */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD     0       /* ATK-MS901M�����ϴ�֡ID */
#define ATK_MS901M_FRAME_ID_TYPE_ACK        1       /* ATK-MS901MӦ��֡ID */

/* ��̬�����ݽṹ�� */
typedef struct
{
    float roll;                                     /* ����ǣ���λ���� */
    float pitch;                                    /* �����ǣ���λ���� */
    float yaw;                                      /* ����ǣ���λ���� */
		
} atk_ms901m_attitude_data_t;

/* ��Ԫ�����ݽṹ�� */
typedef struct
{
    float q0;                                       /* Q0 */
    float q1;                                       /* Q1 */
    float q2;                                       /* Q2 */
    float q3;                                       /* Q3 */
} atk_ms901m_quaternion_data_t;

/* ���������ݽṹ�� */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X��ԭʼ���� */
        int16_t y;                                  /* Y��ԭʼ���� */
        int16_t z;                                  /* Z��ԭʼ���� */
    } raw;
    float x;                                        /* X����ת���ʣ���λ��dps */
    float y;                                        /* Y����ת���ʣ���λ��dps */
    float z;                                        /* Z����ת���ʣ���λ��dps */
} atk_ms901m_gyro_data_t;

/* ���ٶȼ����ݽṹ�� */
typedef struct
{
    struct
    {
        int16_t x;                                  /* X��ԭʼ���� */
        int16_t y;                                  /* Y��ԭʼ���� */
        int16_t z;                                  /* Z��ԭʼ���� */
    } raw;
    float x;                                        /* X����ٶȣ���λ��G */
    float y;                                        /* Y����ٶȣ���λ��G */
    float z;                                        /* Z����ٶȣ���λ��G */
} atk_ms901m_accelerometer_data_t;

/* ���������ݽṹ�� */
typedef struct
{
    int16_t x;                                      /* X��ų�ǿ�� */
    int16_t y;                                      /* Y��ų�ǿ�� */
    int16_t z;                                      /* Z��ų�ǿ�� */
    float temperature;                              /* �¶ȣ���λ���� */
} atk_ms901m_magnetometer_data_t;

/* ��ѹ�����ݽṹ�� */
typedef struct
{
    int32_t pressure;                               /* ��ѹ����λ��Pa */
    int32_t altitude;                               /* ���Σ���λ��cm */
    float temperature;                              /* �¶ȣ���λ���� */
} atk_ms901m_barometer_data_t;
/*�������˲�ϵ��*/
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;
/*�������˲������ýǶ�*/
typedef struct {
		Kalman_t Kalman_X;
		Kalman_t Kalman_Y;
		float KalmanAngleY;
		float KalmanAngleX;
} KalmanAngle;
/* ��״̬�ṹ��*/
typedef struct
{
	atk_ms901m_attitude_data_t attitude_data;
	atk_ms901m_gyro_data_t gyro_data;
	atk_ms901m_accelerometer_data_t accelerometer_data;
	KalmanAngle Kalman_Angle;
    struct {
        float x;
        float y;
        float z;
    }displacement;
     struct {
        float x;
        float y;
        float z;
    }velocity;
}IMU_Parameter;
/* �˿����ݽṹ�� */
typedef struct
{
    uint16_t d0;                                    /* �˿�D0���� */
    uint16_t d1;                                    /* �˿�D1���� */
    uint16_t d2;                                    /* �˿�D2���� */
    uint16_t d3;                                    /* �˿�D3���� */
} atk_ms901m_port_data_t;

/* ATK-MS901M LED״̬ö�� */
typedef enum
{
    ATK_MS901M_LED_STATE_ON  = 0x00,                /* LED�ƹر� */
    ATK_MS901M_LED_STATE_OFF = 0x01,                /* LED�ƴ� */
} atk_ms901m_led_state_t;

/* ATK-MS901M�˿�ö�� */
typedef enum
{
    ATK_MS901M_PORT_D0 = 0x00,                      /* �˿�D0 */
    ATK_MS901M_PORT_D1 = 0x01,                      /* �˿�D1 */
    ATK_MS901M_PORT_D2 = 0x02,                      /* �˿�D2 */
    ATK_MS901M_PORT_D3 = 0x03,                      /* �˿�D3 */
} atk_ms901m_port_t;

/* ATK-MS901M�˿�ģʽö�� */
typedef enum
{
    ATK_MS901M_PORT_MODE_ANALOG_INPUT   = 0x00,     /* ģ������ */
    ATK_MS901M_PORT_MODE_INPUT          = 0x01,     /* �������� */
    ATK_MS901M_PORT_MODE_OUTPUT_HIGH    = 0x02,     /* ������ָߵ�ƽ */
    ATK_MS901M_PORT_MODE_OUTPUT_LOW     = 0x03,     /* ������ֵ͵�ƽ */
    ATK_MS901M_PORT_MODE_OUTPUT_PWM     = 0x04,     /* ���PWM */
} atk_ms901m_port_mode_t;

extern IMU_Parameter IMU_Data;
extern IMU_Parameter IMU_Data_Without;

/* ������� */
#define ATK_MS901M_EOK      0                       /* û�д��� */
#define ATK_MS901M_ERROR    1                       /* ���� */
#define ATK_MS901M_EINVAL   2                       /* ���������� */
#define ATK_MS901M_ETIMEOUT 3                       /* ��ʱ���� */

/* �������� */
uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout);                                                                      /* ͨ��֡ID��ȡATK-MS901M�Ĵ��� */
uint8_t atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, uint8_t *dat);                                                                          /* ͨ��֡IDд��ATK-MS901M�Ĵ��� */
uint8_t atk_ms901m_init(uint32_t baudrate);                                                                                                         /* ATK-MS901M��ʼ�� */
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout);                                                        /* ��ȡATK-MS901M��̬������ */
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout);                                                  /* ��ȡATK-MS901M��Ԫ������ */
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout);  /* ��ȡATK-MS901M�����ǡ����ٶȼ����� */
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout);                                            /* ��ȡATK-MS901M���������� */
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout);                                                     /* ��ȡATK-MS901M��ѹ������ */
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *port_dat, uint32_t timeout);                                                                    /* ��ȡATK-MS901M�˿����� */
uint8_t atk_ms901m_get_led_state(atk_ms901m_led_state_t *state, uint32_t timeout);                                                                  /* ��ȡATK-MS901M LED��״̬ */
uint8_t atk_ms901m_set_led_state(atk_ms901m_led_state_t state, uint32_t timeout);                                                                   /* ����ATK-MS901M LED��״̬ */
uint8_t atk_ms901m_get_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout);                                           /* ��ȡATK-MS901Mָ���˿�ģʽ */
uint8_t atk_ms901m_set_port_mode(atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout);                                            /* ����ATK-MS901Mָ���˿�ģʽ */
uint8_t atk_ms901m_get_port_pwm_pulse(atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout);                                                   /* ��ȡATK-MS901Mָ���˿�PWM�ߵ�ƽ�Ŀ��� */
uint8_t atk_ms901m_set_port_pwm_pulse(atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout);                                                    /* ����ATK-MS901Mָ���˿�PWM�ߵ�ƽ�Ŀ��� */
uint8_t atk_ms901m_get_port_pwm_period(atk_ms901m_port_t port, uint16_t *period, uint32_t timeout);                                                 /* ��ȡATK-MS901Mָ���˿�PWM���� */
uint8_t atk_ms901m_set_port_pwm_period(atk_ms901m_port_t port, uint16_t period, uint32_t timeout);                                                  /* ����ATK-MS901Mָ���˿�PWM���� */
void IMU_All_Read(IMU_Parameter *IMU_Data,uint32_t timeout);
void IMU_GET_Data_Without_Referrence(IMU_Parameter *IMU_Data,uint32_t timeout, int times);
#endif
