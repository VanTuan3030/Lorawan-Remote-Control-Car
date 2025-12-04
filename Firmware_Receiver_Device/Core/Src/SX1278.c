/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#include "SX1278.h"
#include <string.h>

uint8_t SX1278_SPIRead(SX1278_t *module, uint8_t addr) {
	uint8_t tmp;
	SX1278_hw_SPICommand(module->hw, addr);
	tmp = SX1278_hw_SPIReadByte(module->hw);
	SX1278_hw_SetNSS(module->hw, 1);
	return tmp;
}

void SX1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t cmd) {
	SX1278_hw_SetNSS(module->hw, 0);
	SX1278_hw_SPICommand(module->hw, addr | 0x80);
	SX1278_hw_SPICommand(module->hw, cmd);
	SX1278_hw_SetNSS(module->hw, 1);
}

void SX1278_SPIBurstRead(SX1278_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length) {
    if (length == 0) { // Không thể đọc 0 byte
        return;
    }
    // Đọc nhiều byte hoặc một byte đều dùng chung logic này
    SX1278_hw_SetNSS(module->hw, 0);            // Kéo NSS xuống thấp
    SX1278_hw_SPICommand(module->hw, addr);     // Gửi địa chỉ thanh ghi với bit MSB=0 (chế độ đọc)
                                                // SX1278_hw_SPICommand sẽ giữ NSS ở mức thấp

    for (uint8_t i = 0; i < length; i++) {
        rxBuf[i] = SX1278_hw_SPIReadByte(module->hw); // Đọc từng byte dữ liệu
                                                      // SX1278_hw_SPIReadByte sẽ giữ NSS ở mức thấp
    }
    SX1278_hw_SetNSS(module->hw, 1);            // Kéo NSS lên cao
}

void SX1278_SPIBurstWrite(SX1278_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length) {
    if (length == 0) { // Không thể ghi 0 byte
        return;
    }
    // Ghi nhiều byte hoặc một byte đều dùng chung logic này
    SX1278_hw_SetNSS(module->hw, 0);                // Kéo NSS xuống thấp để bắt đầu giao tiếp SPI
    SX1278_hw_SPICommand(module->hw, addr | 0x80);  // Gửi địa chỉ thanh ghi với bit MSB=1 (chế độ ghi)
                                                    // SX1278_hw_SPICommand sẽ giữ NSS ở mức thấp

    for (uint8_t i = 0; i < length; i++) {
        SX1278_hw_SPICommand(module->hw, txBuf[i]); // Gửi từng byte dữ liệu
                                                    // SX1278_hw_SPICommand sẽ giữ NSS ở mức thấp
    }
    SX1278_hw_SetNSS(module->hw, 1);                // Kéo NSS lên cao để kết thúc giao tiếp SPI
}

void SX1278_config(SX1278_t *module) {
	SX1278_sleep(module); //Change modem mode Must in Sleep mode
	SX1278_hw_DelayMs(15);

	SX1278_entryLoRa(module);
	//SX1278_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

	uint64_t freq = ((uint64_t) module->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX1278_SPIBurstWrite(module, LR_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

	SX1278_SPIWrite(module, RegSyncWord, 0x12);

	//setting base parameter
	SX1278_SPIWrite(module, LR_RegPaConfig, SX1278_Power[module->power]); //Setting output power parameter

	SX1278_SPIWrite(module, LR_RegOcp, 0x0B);			//RegOcp,Close Ocp
	SX1278_SPIWrite(module, LR_RegLna, 0x23);		//RegLNA,High & LNA Enable
	if (SX1278_SpreadFactor[module->LoRa_SF] == 6) {	//SFactor=6
		uint8_t tmp;
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

		tmp = SX1278_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1278_SPIWrite(module, 0x31, tmp);
		SX1278_SPIWrite(module, 0x37, 0x0C);
	} else {
		SX1278_SPIWrite(module,
		LR_RegModemConfig1,
				((SX1278_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX1278_CodingRate[module->LoRa_CR] << 1) + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1278_SPIWrite(module,
		LR_RegModemConfig2,
				((SX1278_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX1278_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX1278_SPIWrite(module, LR_RegModemConfig3, 0x04);
	SX1278_SPIWrite(module, LR_RegSymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1278_SPIWrite(module, LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX1278_SPIWrite(module, LR_RegPreambleLsb, 8); //RegPreambleLsb 8+4=12byte Preamble
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytes = 0;
	SX1278_standby(module); //Entry standby mode
}

void SX1278_standby(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x09);
	module->status = STANDBY;
}

void SX1278_sleep(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x08);
	module->status = SLEEP;
}

void SX1278_entryLoRa(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegOpMode, 0x88);
}

void SX1278_clearLoRaIrq(SX1278_t *module) {
	SX1278_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}

int SX1278_LoRaEntryRx(SX1278_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;

	module->packetLength = length;

	SX1278_config(module);		//Setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x84);	//Normal and RX
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0xFF);	//No FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F);//Open RxDone interrupt & Timeout
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegPayloadLength, length);//Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
	addr = SX1278_SPIRead(module, LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX1278_SPIWrite(module, LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytes = 0;

	while (1) {
		if ((SX1278_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {	//Rx-on going RegModemStat
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

uint8_t SX1278_LoRaRxPacket(SX1278_t *module) {
	unsigned char addr;
		unsigned char packet_size = 0; // Khởi tạo packet_size
		uint8_t irq_flags;

		// Kiểm tra xem có ngắt DIO0 không (báo hiệu RxDone)
		// Lưu ý: Cách tốt hơn là xử lý việc đọc gói tin trực tiếp trong EXTI callback
		// hoặc đảm bảo flag loraPacketReadyFlag_RX chỉ được xử lý một lần.
		// Tuy nhiên, với cấu trúc hiện tại, việc kiểm tra lại DIO0 là cần thiết.
		if (SX1278_hw_GetDIO0(module->hw)) {
			irq_flags = SX1278_SPIRead(module, LR_RegIrqFlags); // Đọc thanh ghi cờ ngắt

			// Kiểm tra cờ PayloadCrcError (bit 5) nếu CRC được bật trong cấu hình
			// module->LoRa_CRC_sum == SX1278_LORA_CRC_EN (defined as 0)
			// và SX1278_CRC_Sum[0] là 0x01, nghĩa là bit CRC (bit 2 của LR_RegModemConfig2) được SET.
			if (module->LoRa_CRC_sum == SX1278_LORA_CRC_EN) { // Chỉ kiểm tra nếu CRC được cấu hình là bật
				if (irq_flags & 0x20) { // 0x20 là mặt nạ cho cờ PayloadCrcError (bit 5)
					// Nếu có lỗi CRC:
					SX1278_clearLoRaIrq(module); // Xóa tất cả các cờ ngắt (bao gồm cả CrcError và RxDone)
					module->readBytes = 0;       // Báo không có byte nào hợp lệ được đọc
					// Tùy chọn: Bạn có thể thêm một biến đếm lỗi CRC ở đây để theo dõi
					// Ví dụ: module->crcErrorCount++;
					return 0;                    // Trả về 0 byte, không xử lý gói tin lỗi
				}
			}

			// Nếu không có lỗi CRC (hoặc CRC không được bật), tiếp tục đọc gói tin
			memset(module->rxBuffer, 0x00, SX1278_MAX_PACKET); // Xóa bộ đệm nhận

			addr = SX1278_SPIRead(module, LR_RegFifoRxCurrentaddr); // Đọc địa chỉ con trỏ FIFO hiện tại
			SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); // Đặt con trỏ FIFO tới địa chỉ bắt đầu của gói tin vừa nhận

			if (module->LoRa_SF == SX1278_LORA_SF_6) {
				// Khi Spreading Factor là 6, sử dụng chế độ Implicit Header
				// Độ dài gói tin phải được biết trước (lấy từ module->packetLength đã được cấu hình)
				packet_size = module->packetLength;
			} else {
				// Với các SF khác, sử dụng chế độ Explicit Header
				// Độ dài gói tin được đọc từ thanh ghi LR_RegRxNbBytes
				packet_size = SX1278_SPIRead(module, LR_RegRxNbBytes); // Số byte đã nhận
			}

			// Kiểm tra sơ bộ kích thước gói tin để tránh đọc quá bộ đệm
			if (packet_size > 0 && packet_size <= SX1278_MAX_PACKET) {
				SX1278_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size); // Đọc dữ liệu từ FIFO
				module->readBytes = packet_size;
			} else if (packet_size == 0) {
	            // Trường hợp này có thể xảy ra nếu RxNbBytes trả về 0 dù không có lỗi CRC
	            // (ít khả năng nếu RxDone đã được kích hoạt và không có lỗi)
	            module->readBytes = 0;
	        } else {
				// Kích thước gói tin quá lớn, có thể do lỗi nghiêm trọng
				module->readBytes = 0;
				// Tùy chọn: Ghi nhận lỗi kích thước gói tin không hợp lệ
			}

			SX1278_clearLoRaIrq(module); // Xóa tất cả các cờ ngắt sau khi xử lý
		} else {
			// Nếu DIO0 không ở mức cao khi hàm này được gọi (dù EXTI có thể đã kích hoạt trước đó)
			// Điều này có nghĩa là trạng thái RxDone có thể đã bị xóa hoặc không còn hợp lệ.
			// Để an toàn, không nên xử lý gói tin và coi như không có dữ liệu mới.
			// `module->readBytes` sẽ giữ giá trị cũ hoặc 0 nếu đã được reset bởi SX1278_read().
			// Để đảm bảo, ta có thể set nó về 0 ở đây nếu muốn.
			// Tuy nhiên, do hàm `SX1278_read()` sẽ reset `module->readBytes = 0;`
			// nên việc không thay đổi ở đây thường là ổn.
			// Nếu bạn muốn chắc chắn hơn:
			// module->readBytes = 0; // Uncomment nếu muốn đảm bảo không có byte "cũ" nào được xử lý
		}
		return module->readBytes;
}

int SX1278_LoRaEntryTx(SX1278_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	SX1278_config(module); //setting base parameter
	SX1278_SPIWrite(module, REG_LR_PADAC, 0x87);	//Tx for 20dBm
	SX1278_SPIWrite(module, LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1278_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX1278_clearLoRaIrq(module);
	SX1278_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1278_SPIWrite(module, LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX1278_SPIRead(module, LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX1278_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1) {
		temp = SX1278_SPIRead(module, LR_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
	}
}

int SX1278_LoRaTxPacket(SX1278_t *module, uint8_t *txBuffer, uint8_t length,
		uint32_t timeout) {
	SX1278_SPIBurstWrite(module, 0x00, txBuffer, length);
	SX1278_SPIWrite(module, LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		if (SX1278_hw_GetDIO0(module->hw)) { //if(Get_NIRQ()) //Packet send over
			SX1278_SPIRead(module, LR_RegIrqFlags);
			SX1278_clearLoRaIrq(module); //Clear irq
			SX1278_standby(module); //Entry Standby mode
			return 1;
		}

		if (--timeout == 0) {
			SX1278_hw_Reset(module->hw);
			SX1278_config(module);
			return 0;
		}
		SX1278_hw_DelayMs(1);
	}
}

void SX1278_init(SX1278_t *module, uint64_t frequency, uint8_t power,
		uint8_t LoRa_SF, uint8_t LoRa_BW, uint8_t LoRa_CR,
		uint8_t LoRa_CRC_sum, uint8_t packetLength) {
	SX1278_hw_init(module->hw);
	module->frequency = frequency;
	module->power = power;
	module->LoRa_SF = LoRa_SF;
	module->LoRa_BW = LoRa_BW;
	module->LoRa_CR = LoRa_CR;
	module->LoRa_CRC_sum = LoRa_CRC_sum;
	module->packetLength = packetLength;
	SX1278_config(module);
}

int SX1278_transmit(SX1278_t *module, uint8_t *txBuf, uint8_t length,
		uint32_t timeout) {
	if (SX1278_LoRaEntryTx(module, length, timeout)) {
		return SX1278_LoRaTxPacket(module, txBuf, length, timeout);
	}
	return 0;
}

int SX1278_receive(SX1278_t *module, uint8_t length, uint32_t timeout) {
	return SX1278_LoRaEntryRx(module, length, timeout);
}

uint8_t SX1278_available(SX1278_t *module) {
	return SX1278_LoRaRxPacket(module);
}

uint8_t SX1278_read(SX1278_t *module, uint8_t *rxBuf, uint8_t length) {
	if (length != module->readBytes)
		length = module->readBytes;
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytes = 0;
	return length;
}

uint8_t SX1278_RSSI_LoRa(SX1278_t *module) {
	uint32_t temp = 10;
	temp = SX1278_SPIRead(module, LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX1278_RSSI(SX1278_t *module) {
	uint8_t temp = 0xff;
	temp = SX1278_SPIRead(module, RegRssiValue);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}
