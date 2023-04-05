/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_ADDRESS 0x36
#define AS5600_RAW_ADD_1 0x0D	// bit 7:0
#define AS5600_RAW_ADD_2 0x0C	// bit 11:8
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t raw_angle_1[2];
uint8_t raw_angle_2[2];
uint8_t raw_angle_3[2];

uint16_t angle_1;
uint16_t angle_2;
uint16_t angle_3;
/* USER CODE END PV */


int main()
{
    while(1)
    {
    /* USER CODE BEGIN 3 */
        HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDRESS<<1, AS5600_RAW_ADD_1, 1, &raw_angle_1[0], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDRESS<<1, AS5600_RAW_ADD_2, 1, &raw_angle_1[1], 1, HAL_MAX_DELAY);

        HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS<<1, AS5600_RAW_ADD_1, 1, &raw_angle_2[0], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS<<1, AS5600_RAW_ADD_2, 1, &raw_angle_2[1], 1, HAL_MAX_DELAY);

        HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS<<1, AS5600_RAW_ADD_1, 1, &raw_angle_3[0], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDRESS<<1, AS5600_RAW_ADD_2, 1, &raw_angle_3[1], 1, HAL_MAX_DELAY);

        angle_1 = raw_angle_1[1] << 8 | raw_angle_1[0] * 0.087890625;
        angle_2 = raw_angle_2[1] << 8 | raw_angle_2[0] * 0.087890625;
        angle_3 = raw_angle_3[1] << 8 | raw_angle_3[0] * 0.087890625;
    }
  /* USER CODE END 3 */
}
