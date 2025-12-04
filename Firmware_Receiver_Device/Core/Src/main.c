/******************************************************************************
 * File:   main.c
 * Author: Gemini AI (Dựa trên yêu cầu của người dùng)
 *
 * @brief  Chương trình chính cho xe điều khiển bằng LoRa.
 * - Nhận dữ liệu liên tục từ module LoRa RA-02.
 * - Điều khiển 2 động cơ qua driver L298N.
 * - Xe sẽ chạy theo lệnh cuối cùng nhận được cho đến khi có lệnh mới.
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SX1278.h"
#include "SX1278_hw.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Tham số LoRa (PHẢI KHỚP VỚI BÊN PHÁT) ---
#define LORA_FREQUENCY_HZ       433000000UL
#define LORA_POWER              SX1278_POWER_17DBM
#define LORA_SF                 SX1278_LORA_SF_7
#define LORA_BANDWIDTH          SX1278_LORA_BW_125KHZ
#define LORA_CODING_RATE        SX1278_LORA_CR_4_5
#define LORA_CRC_ENABLE         SX1278_LORA_CRC_EN
#define LORA_EXPLICIT_SYNCWORD  0x12
#define LORA_PACKET_LEN_CONFIG  64

// --- Định nghĩa chân điều khiển Motor L298N ---
// Motor 1 (bánh trái, điều khiển bởi ENA)
#define IN1_GPIO_Port           GPIOB
#define IN1_Pin                 GPIO_PIN_8
#define IN2_GPIO_Port           GPIOB
#define IN2_Pin                 GPIO_PIN_9
#define MOTOR1_PWM_CHANNEL      TIM_CHANNEL_1 // PA0 (nối với ENA)

// Motor 2 (bánh phải, điều khiển bởi ENB)
#define IN3_GPIO_Port           GPIOA
#define IN3_Pin                 GPIO_PIN_2
#define IN4_GPIO_Port           GPIOA
#define IN4_Pin                 GPIO_PIN_3
#define MOTOR2_PWM_CHANNEL      TIM_CHANNEL_2 // PA1 (nối với ENB)

// --- Các mức tốc độ PWM (giả sử ARR của Timer là 999) ---
#define SPEED_FULL              999
#define SPEED_RUN               200 // Tốc độ chạy tiến/lùi
#define SPEED_TURN              150 // Tốc độ khi xoay/rẽ
#define SPEED_STOP              0   // Dừng

// --- Định nghĩa chân LED báo trạng thái ---
#define LED_FORWARD_GPIO_Port   GPIOB
#define LED_FORWARD_Pin         GPIO_PIN_13
#define LED_BACKWARD_GPIO_Port  GPIOB
#define LED_BACKWARD_Pin        GPIO_PIN_14
#define LED_TURN_GPIO_Port      GPIOB
#define LED_TURN_Pin            GPIO_PIN_12
#define LED_LORA_OK_GPIO_Port   GPIOB
#define LED_LORA_OK_Pin         GPIO_PIN_12 // Dùng chung LED báo rẽ để báo LoRa OK
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Các biến này thường được CubeMX tự động tạo ra trong file tương ứng (spi.c, tim.c)
// extern SPI_HandleTypeDef hspi1;
// extern TIM_HandleTypeDef htim2;

// Biến cho chương trình LoRa và motor
SX1278_t LoRaModule_RX;
SX1278_hw_t LoRa_HW_RX;
uint8_t receivedLoRaDataBuffer[LORA_PACKET_LEN_CONFIG];
volatile uint8_t loraPacketReadyFlag_RX = 0; // Cờ báo có tín hiệu LoRa mới

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Khai báo các hàm sẽ được định nghĩa ở cuối file
void Turn_Off_All_LEDs(void);
void Indicate_LoRa_Init_Status(int success);
void motor1_set_speed(uint16_t speed_pulse);
void motor2_set_speed(uint16_t speed_pulse);
void motor1_forward(uint16_t speed);
void motor1_backward(uint16_t speed);
void motor1_stop(void);
void motor2_forward(uint16_t speed);
void motor2_backward(uint16_t speed);
void motor2_stop(void);
void all_motors_forward(uint16_t speed);
void all_motors_backward(uint16_t speed);
void all_motors_stop(void);
void car_turn_right(uint16_t speed);
void car_turn_left(uint16_t speed);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  Turn_Off_All_LEDs();

  // 1. Cấu hình phần cứng cho module LoRa
  LoRa_HW_RX.spi      = &hspi1;
  LoRa_HW_RX.nss.port = (void *)LORA_NSS_GPIO_Port;
  LoRa_HW_RX.nss.pin  = LORA_NSS_Pin;
  LoRa_HW_RX.reset.port = (void *)LORA_RST_GPIO_Port;
  LoRa_HW_RX.reset.pin  = LORA_RST_Pin;
  LoRa_HW_RX.dio0.port = (void *)LORA_DIO0_GPIO_Port;
  LoRa_HW_RX.dio0.pin  = LORA_DIO0_Pin;
  LoRaModule_RX.hw    = &LoRa_HW_RX;

  // 2. Khởi tạo module LoRa
  SX1278_hw_Reset(&LoRa_HW_RX);
  SX1278_init(&LoRaModule_RX,
              LORA_FREQUENCY_HZ,
              LORA_POWER,
              LORA_SF,
              LORA_BANDWIDTH,
              LORA_CODING_RATE,
              LORA_CRC_ENABLE,
              LORA_PACKET_LEN_CONFIG);

  SX1278_SPIWrite(&LoRaModule_RX, RegSyncWord, LORA_EXPLICIT_SYNCWORD);

  // 3. Kiểm tra và báo hiệu trạng thái khởi tạo LoRa
  uint8_t version = SX1278_SPIRead(&LoRaModule_RX, REG_LR_VERSION);
  Indicate_LoRa_Init_Status(version == 0x12);
  HAL_Delay(1000);
  Turn_Off_All_LEDs();

  // 4. Khởi động các kênh PWM cho động cơ
  if (HAL_TIM_PWM_Start(&htim2, MOTOR1_PWM_CHANNEL) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim2, MOTOR2_PWM_CHANNEL) != HAL_OK) Error_Handler();

  // Đảm bảo động cơ dừng hoàn toàn khi bắt đầu
  all_motors_stop();

  // 5. Đặt LoRa vào chế độ nhận liên tục
  int rx_entry_status = SX1278_receive(&LoRaModule_RX, LORA_PACKET_LEN_CONFIG, SX1278_DEFAULT_TIMEOUT);
  if (!rx_entry_status)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Chỉ xử lý khi có ngắt báo hiệu gói tin LoRa đã đến
    if (loraPacketReadyFlag_RX == 1)
    {
      loraPacketReadyFlag_RX = 0; // Xóa cờ ngay để chuẩn bị cho lần ngắt tiếp theo

      uint8_t bytes_in_buffer = SX1278_available(&LoRaModule_RX);

      if (bytes_in_buffer > 0)
      {
        SX1278_read(&LoRaModule_RX, receivedLoRaDataBuffer, bytes_in_buffer);
        uint8_t command = receivedLoRaDataBuffer[0]; // Lấy byte lệnh

        Turn_Off_All_LEDs(); // Tắt hết đèn cũ trước khi thực hiện lệnh mới

        // Xử lý lệnh điều khiển
        switch (command)
        {
          case 0: // Lệnh 0: Dừng xe
            all_motors_stop();
            break;

          case 1: // Lệnh 1: Rẽ phải
            car_turn_right(SPEED_TURN);
            HAL_GPIO_WritePin(LED_TURN_GPIO_Port, LED_TURN_Pin, GPIO_PIN_SET);
            break;

          case 2: // Lệnh 2: Rẽ trái
            car_turn_left(SPEED_TURN);
            HAL_GPIO_WritePin(LED_TURN_GPIO_Port, LED_TURN_Pin, GPIO_PIN_SET);
            break;

          case 3: // Lệnh 3: Chạy lùi
            all_motors_backward(SPEED_RUN);
            HAL_GPIO_WritePin(LED_BACKWARD_GPIO_Port, LED_BACKWARD_Pin, GPIO_PIN_SET);
            break;

          case 4: // Lệnh 4: Chạy tiến
            all_motors_forward(SPEED_RUN);
            HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, GPIO_PIN_SET);
            break;

          default: // Nếu nhận lệnh lạ -> Dừng xe để đảm bảo an toàn
            all_motors_stop();
            break;
        }
      }
      // QUAN TRỌNG: Đặt lại LoRa vào chế độ nhận để sẵn sàng cho gói tin tiếp theo
      rx_entry_status = SX1278_receive(&LoRaModule_RX, LORA_PACKET_LEN_CONFIG, SX1278_DEFAULT_TIMEOUT);
      if (!rx_entry_status)
      {
        Error_Handler();
      }
    }
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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief  Hàm callback được tự động gọi khi có ngắt ngoài (EXTI) xảy ra.
  * Chân DIO0 của LoRa được cấu hình để kích hoạt ngắt này khi nhận xong gói tin.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LORA_DIO0_Pin)
  {
    // Đặt cờ báo hiệu cho vòng lặp chính biết rằng có dữ liệu mới
    loraPacketReadyFlag_RX = 1;
  }
}

/**
 * @brief Tắt tất cả các LED báo trạng thái của xe.
 */
void Turn_Off_All_LEDs(void)
{
  HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BACKWARD_GPIO_Port, LED_BACKWARD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_TURN_GPIO_Port, LED_TURN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Báo hiệu trạng thái khởi tạo LoRa bằng LED.
 */
void Indicate_LoRa_Init_Status(int success)
{
  if (success)
  {
    // Khởi tạo thành công: Sáng LED
    HAL_GPIO_WritePin(LED_LORA_OK_GPIO_Port, LED_LORA_OK_Pin, GPIO_PIN_SET);
  }
  else
  {
    // Khởi tạo thất bại: Nháy LED liên tục để báo lỗi nghiêm trọng
    while (1)
    {
      HAL_GPIO_TogglePin(LED_LORA_OK_GPIO_Port, LED_LORA_OK_Pin);
      HAL_Delay(200);
    }
  }
}

// --- Các hàm điều khiển động cơ ---

void motor1_set_speed(uint16_t speed_pulse)
{
  if (speed_pulse > SPEED_FULL) speed_pulse = SPEED_FULL;
  __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_PWM_CHANNEL, speed_pulse);
}

void motor2_set_speed(uint16_t speed_pulse)
{
  if (speed_pulse > SPEED_FULL) speed_pulse = SPEED_FULL;
  __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_PWM_CHANNEL, speed_pulse);
}

// --- Logic điều khiển cho Motor 1 (bánh trái) ---
void motor1_forward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  motor1_set_speed(speed);
}

void motor1_backward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
  motor1_set_speed(speed);
}

void motor1_stop(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  motor1_set_speed(SPEED_STOP);
}

// --- Logic điều khiển cho Motor 2 (bánh phải) ---
void motor2_forward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  motor2_set_speed(speed);
}

void motor2_backward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
  motor2_set_speed(speed);
}

void motor2_stop(void)
{
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  motor2_set_speed(SPEED_STOP);
}

// --- Các hàm điều khiển toàn bộ xe ---
void all_motors_forward(uint16_t speed)
{
  motor1_forward(speed);
  motor2_forward(speed);
}

void all_motors_backward(uint16_t speed)
{
  motor1_backward(speed);
  motor2_backward(speed);
}

void all_motors_stop(void)
{
  motor1_stop();
  motor2_stop();
}

void car_turn_right(uint16_t speed)
{
  // Bánh trái tiến, bánh phải lùi -> xe quay tại chỗ sang phải
  motor1_forward(speed);
  motor2_backward(speed);
}

void car_turn_left(uint16_t speed)
{
  // Bánh trái lùi, bánh phải tiến -> xe quay tại chỗ sang trái
  motor1_backward(speed);
  motor2_forward(speed);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  // Dừng mọi hoạt động và nháy LED báo lỗi nghiêm trọng
  __disable_irq();
  while (1)
  {
      Turn_Off_All_LEDs();
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED_FORWARD_GPIO_Port, LED_FORWARD_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_BACKWARD_GPIO_Port, LED_BACKWARD_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_TURN_GPIO_Port, LED_TURN_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
