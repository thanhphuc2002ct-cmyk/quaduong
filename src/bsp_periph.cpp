#include "bsp_periph.h"
#include "master.h"

#define DATA_LENGTH 64
static Master master;
static uint8_t periph_data[64];
static uint8_t data_send[64] = {0};
static uint8_t periph_data_index = 0, get_key = 0;


static inline uint8_t IR_Decode17(uint16_t ir_data) 
{
    switch(ir_data) 
    {
        case Key_1:
            return '1';
            break;
        case Key_2:
            return '2';
            break;
        case Key_3:
            return '3'; 
            break;
        case Key_4:
            return '4';
            break;
        case Key_5:
            return '5';
            break;
        case Key_6:
            return '6';
            break;
        case Key_7:
            return '7';
            break;
        case Key_8:
            return '8';
            break;
        case Key_9:
            return '9';
            break;
        case Key_0:
            return '0';
            break;
        case Key_Asterisk:
            return '*';
            break;
        case Key_Hastag:
            return '#';
            break;
        case Key_Up:
            return 'U';
            break;
        case Key_Down:
            return 'D';
            break;
        case Key_Left:
            return 'L';
            break;
        case Key_Right:
            return 'R';
            break;
        case Key_OK:
            return 'O';
            break;
        default:
            break;
    } 
    return 0; // Return 0 if the key is not recognized
}

void Peripheral::Periph_Initialize(uint8_t peripheral_id, Peripheral_InitState state) 
{
    // Code to enable the specified peripheral
    // This is a placeholder function and should be implemented based on the specific hardware requirements
    data_send[0] = '$';
    data_send[1] = 2;
    data_send[2] = state;
    data_send[3] = peripheral_id;
    data_send[63] = '#';
    master.UARTsend(Serial1, data_send, DATA_LENGTH); // Send the initialization data to the slave device
}

void Peripheral::Servo_SetAngle(ServoID servo_id, uint8_t angle) 
{
    if (angle > 180) 
    {
        angle = 180; // Limit the angle to 180 degrees
    }

    uint16_t mapped_angle = 500 + (angle * 2000) / 180; // Map the angle to a value between 1000 and 2000 for transmission
    
    data_send[1] = 4;
    data_send[2] = CMD;
    data_send[3] = servo;
    data_send[4] = servo_id;
    data_send[5] = (mapped_angle >> 8) & 0xFF; // High byte of the mapped angle
    data_send[6] = mapped_angle & 0xFF; // Low byte of the mapped angle
    master.UARTsend(Serial1, data_send, DATA_LENGTH); // Send the servo control data to the slave device at address 0x30
}

void Peripheral::OutputControlRGB(RGB_State state) 
{
    data_send[1] = 3;
    data_send[2] = CMD;
    data_send[3] = WS2812B;
    data_send[4] = state;
    master.UARTsend(Serial1, data_send, DATA_LENGTH); // Send the RGB control data to the slave device
}

void Peripheral::SetRGBColor(uint8_t r, uint8_t g, uint8_t b) 
{
    data_send[1] = 5;
    data_send[2] = CMD;
    data_send[3] = WS2812B;
    data_send[4] = r;
    data_send[5] = g;
    data_send[6] = b;
    master.UARTsend(Serial1, data_send, DATA_LENGTH); // Send the RGB control data to the slave device
}

void Peripheral::Buzzer_On() 
{
    data_send[1] = 3;
    data_send[2] = CMD;
    data_send[3] = Buzzer;
    data_send[4] = ON;
    master.UARTsend(Serial1, data_send, DATA_LENGTH); // Send the buzzer control data to the slave device
}

static inline void periph_get_serial_data() 
{
    static uint8_t index = 0; 
    
    while(master.UARTavailable(Serial1)) 
    {
        uint8_t c = master.UARTread(Serial1);
        
        // MẸO DEBUG: Nếu muốn biết mạch có nhận được tín hiệu hay không, hãy bỏ // ở dòng dưới
        // Serial.print((char)c); 

        if (c == '$') { 
            index = 0; // Bắt đầu một gói tin mới
        }
        
        periph_data[index++] = c;

        if (c == '#') { 
            periph_data_index = index; 
            get_key = 1; // <--- CHỈ BÁO HIỆU KHI ĐÃ NHẬN TRỌN VẸN GÓI TIN TỪ MẠCH SLAVE
            index = 0; 
            break; 
        }
        
        if (index >= 64) index = 0; // Chống tràn RAM
    }
}

uint8_t Peripheral::Get_IR_Code()
{
    periph_get_serial_data();

    if(get_key)
    {
        get_key = 0; // SỬA QUAN TRỌNG: Reset cờ ngay lập tức để tránh giải mã lặp lại 1 phím cũ
        
        if(periph_data[0] == '$' && periph_data[periph_data_index - 1] == '#')
        {
            if(periph_data[1] == DATA && periph_data[2] == IR) {
                uint8_t decoded_key = IR_Decode17((periph_data[5] << 8) | periph_data[6]);
                
                // MẸO DEBUG: Bỏ // ở 2 dòng dưới để in ra phím bấm thô
                // Serial.print("Phim IR da giai ma: ");
                // Serial.println((char)decoded_key);  
                
                return decoded_key;
            }
        }
    }
    
    return 0; 
}