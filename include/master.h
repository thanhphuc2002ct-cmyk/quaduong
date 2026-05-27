#ifndef MASTER_H
#define MASTER_H

#include "Wire.h"
#include "HardwareSerial.h"
#include "stdint.h"

class Master {
    public:
        void beginI2C(uint8_t sdaPin, uint8_t sclPin);
        void beginI2C(uint8_t sdaPin, uint8_t sclPin, uint32_t frequency);
        void beginUART(HardwareSerial& serial, uint8_t TxPin, uint8_t RxPin, uint32_t baudrate);
        void I2CrequestFrom(uint8_t address, uint8_t quantity, uint8_t *rx_data);
        void I2CsendTo(uint8_t address, uint8_t *data, uint8_t quantity);
        void I2CsendStr(uint8_t address, String tx_data);
        int UARTavailable(HardwareSerial& serial);
        void UARTsend(HardwareSerial& serial, uint8_t *tx_data, uint8_t quantity) ;
        uint8_t UARTread(HardwareSerial& serial);
    private:
        uint8_t _address;
};
#endif // MASTER_H