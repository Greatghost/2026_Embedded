#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "main.h"
#include "cmsis_os.h"
#include "tools.h"
extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{

}

void BMI088_com_init(void)
{


}

void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}




void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000) != HAL_OK)
    {
        return 0;
    }
    return rx_data;
}

/**
 * @brief  运行期读取两颗BMI088芯片的WHO_AM_I
 * @retval BMI088_ACCEL_ONLINE | BMI088_GYRO_ONLINE 位掩码
 * @note   由INS任务低频调用，避免把“任务仍在运行”误当作“传感器在线”。
 */
uint8_t BMI088_CheckOnline(void)
{
    uint8_t online = 0;
    uint8_t chip_id;

    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte(BMI088_ACC_CHIP_ID | 0x80u);
    BMI088_read_write_byte(0x55u); /* 加速度计SPI读取需要一个dummy byte */
    chip_id = BMI088_read_write_byte(0x55u);
    BMI088_ACCEL_NS_H();
    if (chip_id == BMI088_ACC_CHIP_ID_VALUE)
    {
        online |= BMI088_ACCEL_ONLINE;
    }

    BMI088_GYRO_NS_L();
    BMI088_read_write_byte(BMI088_GYRO_CHIP_ID | 0x80u);
    chip_id = BMI088_read_write_byte(0x55u);
    BMI088_GYRO_NS_H();
    if (chip_id == BMI088_GYRO_CHIP_ID_VALUE)
    {
        online |= BMI088_GYRO_ONLINE;
    }

    return online;
}
