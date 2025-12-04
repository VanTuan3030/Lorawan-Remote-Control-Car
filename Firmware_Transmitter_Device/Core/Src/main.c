/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax, Ay, Az;
} MPU6050_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- MPU6050 Defines ---
#define MPU6050_I2C_ADDR        (0x68 << 1)
#define MPU6050_REG_WHO_AM_I    0x75
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// --- LED Defines ---
// LED PB12 cho trục Xoay (Roll)
#ifndef MPU_LED_X_Pin
#define MPU_LED_X_Pin           GPIO_PIN_12
#define MPU_LED_X_GPIO_Port     GPIOB
#endif
// LED PB13 cho Cúi xuống (Pitch Down)
#ifndef MPU_LED_Y_Pin
#define MPU_LED_Y_Pin           GPIO_PIN_13
#define MPU_LED_Y_GPIO_Port     GPIOB
#endif
// LED PB14 cho Ngửa lên (Pitch Up)
#ifndef MPU_LED_Z_Pin
#define MPU_LED_Z_Pin           GPIO_PIN_14
#define MPU_LED_Z_GPIO_Port     GPIOB
#endif

// LED trạng thái chung trên chân PC13
#ifndef STATUS_LED_Pin
#define STATUS_LED_Pin          GPIO_PIN_13
#define STATUS_LED_GPIO_Port    GPIOC
#endif

// --- Ngưỡng logic MPU ---
// Khi đặt trên mặt bàn (phẳng)
#define FLAT_AZ_MIN_ABS      0.85f // Gia tốc Z (tuyệt đối) phải LỚN HƠN
#define FLAT_AXY_MAX_ABS     0.30f // Gia tốc X và Y (tuyệt đối) phải NHỎ HƠN

// Khi xoay/nghiêng
#define TILT_AX_RIGHT_THRESHOLD  0.45f  // Xoay phải: Ax > ngưỡng
#define TILT_AX_LEFT_THRESHOLD  -0.45f  // Xoay trái: Ax < ngưỡng
#define TILT_AY_UP_THRESHOLD     0.45f  // Ngửa lên: Ay > ngưỡng
#define TILT_AY_DOWN_THRESHOLD  -0.45f  // Cúi xuống: Ay < ngưỡng

// Để xác định nghiêng rõ ràng, các trục khác không nên quá lớn
#define OTHER_AXES_MAX_ABS_FOR_CLEAR_TILT 0.75f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  ITM_SendChar(ch);
  return ch;
}

// --- MPU6050 Functions Definitions ---
HAL_StatusTypeDef MPU6050_Write_Reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef MPU6050_Read_Reg(uint8_t reg, uint8_t *data) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef MPU6050_Read_Multi_Reg(uint8_t reg, uint8_t *data, uint16_t count) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, count, HAL_MAX_DELAY);
}
HAL_StatusTypeDef MPU6050_Init_Device(void) {
    uint8_t check_val; HAL_StatusTypeDef status;
    status = MPU6050_Read_Reg(MPU6050_REG_WHO_AM_I, &check_val);
    if (status != HAL_OK) return HAL_ERROR;
    if (check_val != 0x68) return HAL_ERROR;
    status = MPU6050_Write_Reg(MPU6050_REG_PWR_MGMT_1, 0x00); if (status != HAL_OK) return HAL_ERROR;
    HAL_Delay(10);
    status = MPU6050_Write_Reg(MPU6050_REG_SMPLRT_DIV, 0x07); if (status != HAL_OK) return HAL_ERROR;
    status = MPU6050_Write_Reg(MPU6050_REG_ACCEL_CONFIG, 0x00); if (status != HAL_OK) return HAL_ERROR; // +/- 2g
    status = MPU6050_Write_Reg(MPU6050_REG_GYRO_CONFIG, 0x00); if (status != HAL_OK) return HAL_ERROR;   // +/- 250 deg/s
    return HAL_OK;
}
HAL_StatusTypeDef MPU6050_Read_Accel_Raw(MPU6050_Data_t *data_struct) {
    uint8_t rec_data[6]; HAL_StatusTypeDef status;
    status = MPU6050_Read_Multi_Reg(MPU6050_REG_ACCEL_XOUT_H, rec_data, 6);
    if (status != HAL_OK) { return HAL_ERROR; }
    data_struct->Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    data_struct->Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    data_struct->Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);
    return HAL_OK;
}
void MPU6050_Convert_To_G(MPU6050_Data_t *data_struct) {
    float accel_sensitivity = 16384.0f; // Độ nhạy cho thang đo +/- 2g
    data_struct->Ax = data_struct->Accel_X_RAW / accel_sensitivity;
    data_struct->Ay = data_struct->Accel_Y_RAW / accel_sensitivity;
    data_struct->Az = data_struct->Accel_Z_RAW / accel_sensitivity;
}

// --- LED Control Functions ---
void Turn_Off_MPU_LEDs(void) {
  HAL_GPIO_WritePin(MPU_LED_X_GPIO_Port, MPU_LED_X_Pin, GPIO_PIN_RESET); // PB12
  HAL_GPIO_WritePin(MPU_LED_Y_GPIO_Port, MPU_LED_Y_Pin, GPIO_PIN_RESET); // PB13
  HAL_GPIO_WritePin(MPU_LED_Z_GPIO_Port, MPU_LED_Z_Pin, GPIO_PIN_RESET); // PB14
}

// --- Status Indication Function ---
void Indicate_Overall_Status(int status_code) {
  #ifdef STATUS_LED_Pin
  switch(status_code) {
    case 0: // Lỗi nghiêm trọng -> Nháy chậm liên tục
      while (1) {
        HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); HAL_Delay(500);
      }
      break;
    case 1: // Khởi tạo OK -> Sáng 0.5s rồi tắt
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET); HAL_Delay(500);
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
      break;
    case 2: // Đang gửi TX -> Chớp nhanh 1 lần
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET); HAL_Delay(30);
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
      break;
    case 3: // Gửi TX thất bại -> Chớp nhanh 3 lần
      for(int i=0; i<3; i++){ HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); HAL_Delay(80); HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); HAL_Delay(80); }
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
      break;
    case 4: // Đang thử lại MPU Init -> Chớp 2 lần
      for (int k = 0; k < 2; k++) { HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); HAL_Delay(120); HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin); HAL_Delay(120); }
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
      break;
  }
  #endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("System Initialized. MPU6050 + LoRa TX - Combined Tilt Logic.\r\n");

  #ifdef STATUS_LED_Pin
    // Giả định LED nối với VCC, set PIN_SET để tắt
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  #endif
  Turn_Off_MPU_LEDs();

  // --- KHỞI TẠO MPU6050 ---
  printf("Locating MPU6050...\r\n");
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_I2C_ADDR, 3, 100) != HAL_OK) {
      printf("FATAL: MPU6050 NOT FOUND ON I2C BUS.\r\n");
      system_status_flag = 1; // MPU not found
      Indicate_Overall_Status(0); // Dừng với lỗi nghiêm trọng
  }

  printf("MPU6050 found. Initializing device...\r\n");
  while (MPU6050_Init_Device() != HAL_OK && system_status_flag == 0) {
      system_status_flag = 2; // MPU Init Retrying
      printf("MPU6050 Init Failed. Retrying in 2s...\r\n");
      Indicate_Overall_Status(4); // Nháy LED báo thử lại
      HAL_Delay(2000);
  }
  if (system_status_flag == 0) {
    system_status_flag = 0; // Reset lại cờ nếu trước đó là 2
    printf("MPU6050 Initialized Successfully.\r\n");
    Indicate_Overall_Status(1); // Báo khởi tạo thành công
  }


  // --- KHỞI TẠO LORA ---
  if (system_status_flag == 0) {
    printf("Initializing LoRa TX Module...\r\n");
    LoRa_HW_TX.spi = &hspi1;
    LoRa_HW_TX.nss.port = LORA_NSS_GPIO_Port; LoRa_HW_TX.nss.pin = LORA_NSS_Pin;
    LoRa_HW_TX.reset.port = LORA_RST_GPIO_Port; LoRa_HW_TX.reset.pin = LORA_RST_Pin;
    LoRaModule_TX.hw = &LoRa_HW_TX;

    SX1278_hw_Reset(&LoRa_HW_TX);
    SX1278_init(&LoRaModule_TX,
                LORA_FREQUENCY_HZ, LORA_TX_POWER, LORA_SF,
                LORA_BANDWIDTH, LORA_CODING_RATE, LORA_CRC_ENABLE,
                LORA_PACKET_MAX_LEN);
    uint8_t version = SX1278_SPIRead(&LoRaModule_TX, REG_LR_VERSION);

    if (version == 0x12) {
      printf("LoRa TX Module Initialized. Version: 0x%02X\r\n", version);
      Indicate_Overall_Status(1);
    } else {
      printf("LoRa TX Module Init FAILED! Read Version: 0x%02X\r\n", version);
      system_status_flag = 3; // LoRa init failed
      Indicate_Overall_Status(0); // Dừng với lỗi nghiêm trọng
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (system_status_flag == 0) { // Chỉ hoạt động nếu cả MPU và LoRa đều OK
        if (MPU6050_Read_Accel_Raw(&mpu_data) == HAL_OK) {
            MPU6050_Convert_To_G(&mpu_data);

            Turn_Off_MPU_LEDs(); // Tắt hết LED trước khi kiểm tra
            data_to_send_lora = 0; // Mặc định là trạng thái 0

            float ax = mpu_data.Ax;
            float ay = mpu_data.Ay;
            float az = mpu_data.Az;
            float abs_ax = fabs(ax);
            float abs_ay = fabs(ay);
            float abs_az = fabs(az);

            // 1. KIỂM TRA TRẠNG THÁI "PHẲNG"
            if (abs_az > FLAT_AZ_MIN_ABS && abs_ax < FLAT_AXY_MAX_ABS && abs_ay < FLAT_AXY_MAX_ABS) {
                data_to_send_lora = 0;
            }

            }
            // 3. KIỂM TRA CÁC TRẠNG THÁI RIÊNG LẺ
            // Chỉ xoay phải
            else if (ax > TILT_AX_RIGHT_THRESHOLD && abs_ay < OTHER_AXES_MAX_ABS_FOR_CLEAR_TILT) {
                HAL_GPIO_WritePin(MPU_LED_X_GPIO_Port, MPU_LED_X_Pin, GPIO_PIN_SET);
                data_to_send_lora = 1;
            }
            // Chỉ xoay trái
            else if (ax < TILT_AX_LEFT_THRESHOLD && abs_ay < OTHER_AXES_MAX_ABS_FOR_CLEAR_TILT) {
                HAL_GPIO_WritePin(MPU_LED_X_GPIO_Port, MPU_LED_X_Pin, GPIO_PIN_SET);
                data_to_send_lora = 2;
            }
            // Chỉ ngửa lên
            else if (ay > TILT_AY_UP_THRESHOLD && abs_ax < OTHER_AXES_MAX_ABS_FOR_CLEAR_TILT) {
                HAL_GPIO_WritePin(MPU_LED_Z_GPIO_Port, MPU_LED_Z_Pin, GPIO_PIN_SET);
                data_to_send_lora = 3;
            }
            // Chỉ cúi xuống
            else if (ay < TILT_AY_DOWN_THRESHOLD && abs_ax < OTHER_AXES_MAX_ABS_FOR_CLEAR_TILT) {
                HAL_GPIO_WritePin(MPU_LED_Y_GPIO_Port, MPU_LED_Y_Pin, GPIO_PIN_SET);
                data_to_send_lora = 4;
            }

            // GỬI DỮ LIỆU QUA LORA (Gửi trong mọi trường hợp)
            printf("State Detected: %d -> Sending over LoRa...\r\n", data_to_send_lora);
            Indicate_Overall_Status(2); // Báo hiệu hành động TX
            lora_tx_status_from_lib = SX1278_transmit(&LoRaModule_TX, &data_to_send_lora, 1, 200);

            if (lora_tx_status_from_lib != 1) {
                printf("LoRa Packet Send FAILED! Lib Status: %d\r\n", lora_tx_status_from_lib);
                Indicate_Overall_Status(3); // Báo hiệu TX Fail
            }

        } else { // Lỗi đọc MPU
            Turn_Off_MPU_LEDs();
            HAL_Delay(100);
        }

        HAL_Delay(200); // Delay giữa các lần đọc và gửi
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Các hàm hỗ trợ đã được chuyển lên USER CODE 0
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("FATAL SYSTEM ERROR in Error_Handler().\r\n");
  __disable_irq();
  while (1) {
      #ifdef STATUS_LED_Pin
      // Nháy cực nhanh để báo lỗi hệ thống
      HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
      HAL_Delay(50);
      #else
      HAL_Delay(100);
      #endif
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
