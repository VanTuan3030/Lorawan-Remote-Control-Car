// File: MPU6050.c

#include "MPU6050.h"
#include "main.h"  // Để có GPIO_PIN_SET/RESET, HAL_Delay, và các định nghĩa HAL khác
#include "i2c.h"   // Để có extern I2C_HandleTypeDef hi2c1; (hoặc tên handle I2C của bạn)
#include <stdio.h> // Cho các hàm printf (cần retarget printf để hoạt động)

// Giả định rằng handle I2C của bạn là 'hi2c1'. Nếu khác, hãy thay đổi nó trong các hàm I2C bên dưới.
// extern I2C_HandleTypeDef hi2c1; // Đảm bảo dòng này có trong i2c.h hoặc bạn khai báo nó ở đây nếu cần.

// Biến toàn cục để lưu trữ dữ liệu MPU6050, được khai báo trong MPU6050.h là extern
Struct_MPU6050 MPU6050;

// Biến static cho độ nhạy LSB (Least Significant Bit)
static float LSB_Sensitivity_ACC;
static float LSB_Sensitivity_GYRO;

/**
  * @brief  Ghi một byte vào thanh ghi của MPU6050.
  * @param  reg_addr: Địa chỉ thanh ghi.
  * @param  val: Giá trị để ghi.
  * @retval None
  */
void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Ghi nhiều byte vào các thanh ghi liên tiếp của MPU6050.
  * @param  reg_addr: Địa chỉ thanh ghi bắt đầu.
  * @param  len: Số lượng byte để ghi.
  * @param  data: Con trỏ đến buffer chứa dữ liệu.
  * @retval None
  */
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Đọc một byte từ thanh ghi của MPU6050.
  * @param  reg_addr: Địa chỉ thanh ghi.
  * @param  data: Con trỏ để lưu trữ giá trị đọc được.
  * @retval None
  */
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Đọc nhiều byte từ các thanh ghi liên tiếp của MPU6050.
  * @param  reg_addr: Địa chỉ thanh ghi bắt đầu.
  * @param  len: Số lượng byte để đọc.
  * @param  data: Con trỏ đến buffer để lưu trữ dữ liệu đọc được.
  * @retval None
  */
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/**
  * @brief  Khởi tạo cảm biến MPU6050.
  * @retval None
  */
void MPU6050_Initialization(void)
{
    HAL_Delay(50); // Chờ một chút để MPU6050 ổn định sau khi cấp nguồn (có thể tăng thêm nếu cần)

    uint8_t who_am_i = 0;
    // printf("Checking MPU6050...\r\n"); // Cần retarget printf

    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who_am_i); // Đọc thanh ghi WHO_AM_I

    if (who_am_i == 0x68) // Địa chỉ mặc định của MPU6050 là 0x68
    {
        // printf("MPU6050 who_am_i = 0x%02x...OK\r\n", who_am_i);
    }
    else
    {
        // printf("ERROR!\r\n");
        // printf("MPU6050 who_am_i: 0x%02x (expected 0x68)\r\n", who_am_i);
        // Nếu không đọc được WHO_AM_I đúng, có thể dừng ở đây hoặc báo lỗi
        while (1)
        {
            // printf("WHO_AM_I check failed. Check MPU6050 connection or I2C settings.\r\n");
            // Nháy LED hoặc xử lý lỗi khác ở đây nếu muốn thay vì chỉ printf
            HAL_Delay(500);
        }
    }

    // Reset MPU6050
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x80); // (0x1 << 7)
    HAL_Delay(200); // Tăng delay sau reset (khuyến nghị từ 100ms lên 200ms)

    // Đánh thức MPU6050, chọn nguồn clock là Gyro X (ổn định hơn)
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x01); // Thoát khỏi chế độ ngủ, clock = Gyro X
    HAL_Delay(100); // Tăng delay sau khi đánh thức (khuyến nghị từ 50ms lên 100ms)

    // Cấu hình Sample Rate Divider
    // Sample Rate = Gyroscope Output Rate / (1 + SMPRT_DIV)
    // Gyroscope Output Rate là 8kHz khi DLPF tắt (CONFIG_DLPF_CFG = 0 hoặc 7)
    // Gyroscope Output Rate là 1kHz khi DLPF bật (CONFIG_DLPF_CFG = 1 đến 6)
    // Mặc định DLPF trong CONFIG là 0 (Gyro 8kHz).
    // Để có Sample Rate 200Hz với Gyro 8kHz: 8000 / (1+39) = 200Hz
    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // Sample Rate = 200Hz
    HAL_Delay(50);

    // Cấu hình Digital Low Pass Filter (DLPF) và FSYNC
    // DLPF_CFG = 0: Accel BW 260Hz, Delay 0ms; Gyro BW 256Hz, Delay 0.98ms, Fs 8kHz
    // Các giá trị khác cho DLPF_CFG sẽ giảm băng thông và tăng delay, Fs Gyro sẽ là 1kHz
    MPU6050_Writebyte(MPU6050_CONFIG, 0x00); // DLPF_CFG = 0 (Không dùng DLPF hoặc BW rộng nhất)
    HAL_Delay(50);

    // Cấu hình Gyroscope Full Scale Range
    // FS_SEL | Full Scale Range
    // 0      | +/- 250 độ/giây
    // 1      | +/- 500 độ/giây
    // 2      | +/- 1000 độ/giây
    // 3      | +/- 2000 độ/giây
    uint8_t FS_SCALE_GYRO = 0x0; // Chọn +/- 250 độ/giây
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG, FS_SCALE_GYRO << 3);
    HAL_Delay(50);

    // Cấu hình Accelerometer Full Scale Range
    // AFS_SEL | Full Scale Range
    // 0       | +/- 2g
    // 1       | +/- 4g
    // 2       | +/- 8g
    // 3       | +/- 16g
    uint8_t FS_SCALE_ACC = 0x0; // Chọn +/- 2g
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, FS_SCALE_ACC << 3);
    HAL_Delay(50);

    // Lấy độ nhạy LSB dựa trên cấu hình Full Scale Range
    MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
    // printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\r\n", LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);

    // Cấu hình chân Interrupt (INT) (tùy chọn, nếu bạn dùng ngắt)
    // INT_LEVEL      | bit 7 | 0 = active high, 1 = active low
    // LATCH_INT_EN   | bit 5 | 0 = 50us pulse, 1 = latch until cleared
    // INT_RD_CLEAR   | bit 4 | 0 = status cleared by reading INT_STATUS, 1 = status cleared by any read
    uint8_t INT_LEVEL = 0x0;    // Active high
    uint8_t LATCH_INT_EN = 0x0; // 50us pulse
    uint8_t INT_RD_CLEAR = 0x1; // Clear on any read
    MPU6050_Writebyte(MPU6050_INT_PIN_CFG, (INT_LEVEL << 7) | (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4));
    HAL_Delay(50);

    // Kích hoạt ngắt Data Ready (tùy chọn)
    // DATA_RDY_EN | bit 0 | 1 = enable Data Ready interrupt, 0 = disable
    uint8_t DATA_RDY_EN = 0x1; // Enable Data Ready interrupt
    MPU6050_Writebyte(MPU6050_INT_ENABLE, DATA_RDY_EN);
    HAL_Delay(50);

    // printf("MPU6050 setting is finished\r\n");
}

/**
  * @brief  Lấy dữ liệu thô 6 trục (Accel, Gyro) và nhiệt độ từ MPU6050.
  * @param  mpu6050_data: Con trỏ đến struct để lưu trữ dữ liệu.
  * @retval None
  */
void MPU6050_Get6AxisRawData(Struct_MPU6050* mpu6050_data)
{
    uint8_t data_buf[14];
    MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data_buf);

    mpu6050_data->acc_x_raw = (int16_t)((data_buf[0] << 8) | data_buf[1]);
    mpu6050_data->acc_y_raw = (int16_t)((data_buf[2] << 8) | data_buf[3]);
    mpu6050_data->acc_z_raw = (int16_t)((data_buf[4] << 8) | data_buf[5]);

    mpu6050_data->temperature_raw = (int16_t)((data_buf[6] << 8) | data_buf[7]);

    mpu6050_data->gyro_x_raw = (int16_t)((data_buf[8] << 8) | data_buf[9]);
    mpu6050_data->gyro_y_raw = (int16_t)((data_buf[10] << 8) | data_buf[11]);
    mpu6050_data->gyro_z_raw = (int16_t)((data_buf[12] << 8) | data_buf[13]);
}

/**
  * @brief  Tính toán độ nhạy LSB dựa trên cài đặt Full Scale Range.
  * @param  FS_SCALE_GYRO: Cài đặt Full Scale cho Gyro (0-3).
  * @param  FS_SCALE_ACC: Cài đặt Full Scale cho Accel (0-3).
  * @retval None
  */
void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
    switch (FS_SCALE_GYRO)
    {
    case 0: LSB_Sensitivity_GYRO = 131.0f; break; // +/- 250 dps
    case 1: LSB_Sensitivity_GYRO = 65.5f;  break; // +/- 500 dps
    case 2: LSB_Sensitivity_GYRO = 32.8f;  break; // +/- 1000 dps
    case 3: LSB_Sensitivity_GYRO = 16.4f;  break; // +/- 2000 dps
    default: LSB_Sensitivity_GYRO = 131.0f; break;
    }

    switch (FS_SCALE_ACC)
    {
    case 0: LSB_Sensitivity_ACC = 16384.0f; break; // +/- 2g
    case 1: LSB_Sensitivity_ACC = 8192.0f;  break; // +/- 4g
    case 2: LSB_Sensitivity_ACC = 4096.0f;  break; // +/- 8g
    case 3: LSB_Sensitivity_ACC = 2048.0f;  break; // +/- 16g
    default: LSB_Sensitivity_ACC = 16384.0f; break;
    }
}

/**
  * @brief  Chuyển đổi dữ liệu thô sang đơn vị vật lý (g, độ/giây, độ C).
  * @param  mpu6050_data: Con trỏ đến struct chứa dữ liệu thô và để lưu dữ liệu đã chuyển đổi.
  * @retval None
  */
void MPU6050_DataConvert(Struct_MPU6050* mpu6050_data)
{
    mpu6050_data->acc_x = (float)mpu6050_data->acc_x_raw / LSB_Sensitivity_ACC;
    mpu6050_data->acc_y = (float)mpu6050_data->acc_y_raw / LSB_Sensitivity_ACC;
    mpu6050_data->acc_z = (float)mpu6050_data->acc_z_raw / LSB_Sensitivity_ACC;

    // Công thức nhiệt độ: Temperature in degrees C = (TEMP_OUT Register Value / 340) + 36.53
    mpu6050_data->temperature = (float)mpu6050_data->temperature_raw / 340.0f + 36.53f;

    mpu6050_data->gyro_x = (float)mpu6050_data->gyro_x_raw / LSB_Sensitivity_GYRO;
    mpu6050_data->gyro_y = (float)mpu6050_data->gyro_y_raw / LSB_Sensitivity_GYRO;
    mpu6050_data->gyro_z = (float)mpu6050_data->gyro_z_raw / LSB_Sensitivity_GYRO;
}

/**
  * @brief  Kiểm tra xem có dữ liệu mới sẵn sàng từ MPU6050 qua chân INT không.
  * (Yêu cầu chân MPU6050_INT_PIN được cấu hình là Input trong CubeMX).
  * @retval 1 nếu dữ liệu sẵn sàng, 0 nếu không.
  */
int MPU6050_DataReady(void)
{
    // Cách đơn giản là đọc trực tiếp trạng thái chân INT (nếu đã cấu hình active high)
    // Chân INT của MPU6050 là PB5 theo MPU6050.h của bạn
    return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN) == GPIO_PIN_SET;

    // Cách phức tạp hơn là đọc thanh ghi INT_STATUS (nếu INT_PIN_CFG được cấu hình phù hợp)
    /*
    uint8_t int_status;
    MPU6050_Readbyte(MPU6050_INT_STATUS, &int_status);
    if (int_status & 0x01) // Kiểm tra bit DATA_RDY_INT
    {
        return 1; // Dữ liệu sẵn sàng
    }
    return 0; // Dữ liệu chưa sẵn sàng
    */
}

/**
  * @brief  Xử lý toàn bộ dữ liệu MPU6050: đọc dữ liệu thô và chuyển đổi.
  * @param  mpu6050_data: Con trỏ đến struct để lưu trữ dữ liệu.
  * @retval None
  */
void MPU6050_ProcessData(Struct_MPU6050* mpu6050_data)
{
    MPU6050_Get6AxisRawData(mpu6050_data);
    MPU6050_DataConvert(mpu6050_data);
}
