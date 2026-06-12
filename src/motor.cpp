#include "motor.h"
#include "config.h"


void setMotors(int speedLeft, int speedRight) {
    // Thu phóng toàn bộ tốc độ truyền vào theo hệ số trước khi xử lý
    speedLeft = speedLeft * SPEED_SCALE;
    speedRight = speedRight * SPEED_SCALE;

    if (speedLeft > 0) { speedLeft -= 0; if (speedLeft < 0) speedLeft = 0; }
    
    // Bù sai số cơ khí khi lùi: Bánh phải tự quay nhanh hơn nên ta cộng thêm 5 để hãm nó lại (ví dụ từ -80 thành -75)
    if (speedRight < 0) { speedRight += 3; if (speedRight > 0) speedRight = 0; }

    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);

    if (speedLeft > 0) { analogWrite(M3_INA1, 255 - speedLeft); analogWrite(M3_INA2, 255); }
    else if (speedLeft < 0) { analogWrite(M3_INA1, 255); analogWrite(M3_INA2, 255 + speedLeft); }
    else { analogWrite(M3_INA1, 0); analogWrite(M3_INA2, 0); }

    if (speedRight > 0) { analogWrite(M4_INA1, 255 - speedRight); analogWrite(M4_INA2, 255); }
    else if (speedRight < 0) { analogWrite(M4_INA1, 255); analogWrite(M4_INA2, 255 + speedRight); }
    else { analogWrite(M4_INA1, 0); analogWrite(M4_INA2, 0); }
}

void motorInit() {
    pinMode(M3_INA1, OUTPUT); pinMode(M3_INA2, OUTPUT);
    pinMode(M4_INA1, OUTPUT); pinMode(M4_INA2, OUTPUT);
    setMotors(0, 0);
}