
#include "master.h" 
#include "stdlib.h"


void Master::beginI2C(uint8_t sdaPin, uint8_t sclPin) 
{
    Wire1.begin(sdaPin, sclPin);
}

void Master::beginI2C(uint8_t sdaPin, uint8_t sclPin, uint32_t frequency) 
{
    Wire1.begin(sdaPin, sclPin, frequency);
}

void Master::beginUART(HardwareSerial& serial, uint8_t TxPin, uint8_t RxPin, uint32_t baudrate) 
{
    serial.begin(baudrate, SERIAL_8N1, RxPin, TxPin);
}

void Master::I2CrequestFrom(uint8_t address, uint8_t quantity, uint8_t *rx_data) 
{
    Wire1.requestFrom(address, quantity);

    while (Wire1.available()) { // slave may send less than requested
        *rx_data = Wire1.read(); // receive a byte as character
        rx_data++;
    }
}

void Master::I2CsendTo(uint8_t address, uint8_t *tx_data, uint8_t quantity) 
{
    Wire1.beginTransmission(address);
    Wire1.write(tx_data, quantity); // send byte
    // Serial.print("Sent byte: ");
    // Serial.println((char)tx_data[i]);
    uint8_t result = Wire1.endTransmission(); // stop transmitting

    if (result != 0) {
        Serial.print("Error in transmission: ");
        Serial.println(result);
    }
}

void Master::I2CsendStr(uint8_t address, String tx_data)
{
    Wire1.beginTransmission(address);
    Wire1.printf("%s", tx_data.c_str());
    uint8_t result = Wire1.endTransmission(); // stop transmitting

    if (result != 0) {
        Serial.print("Error in transmission: ");
        Serial.println(result);
    }
}

int Master::UARTavailable(HardwareSerial& serial) 
{
    return serial.available();
}

void Master::UARTsend(HardwareSerial& serial, uint8_t *tx_data, uint8_t quantity) 
{
    serial.write(tx_data, quantity);
}

uint8_t Master::UARTread(HardwareSerial& serial) 
{
    return serial.read();
}