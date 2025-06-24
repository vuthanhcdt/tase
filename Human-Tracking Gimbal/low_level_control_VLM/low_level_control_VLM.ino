#include <SimpleFOC.h>

// Pin Definitions
#define SENSOR_CS_PIN 10
#define MOTOR_PWM_PIN1 9
#define MOTOR_PWM_PIN2 5
#define MOTOR_PWM_PIN3 6
#define MOTOR_ENABLE_PIN 8

// Motor Configuration
#define MOTOR_POLE_PAIRS 14
#define VOLTAGE_POWER 24.0
#define VOLTAGE_LIMIT 20.0
#define MOTOR_LIMIT 20.0

#define DATA_LENGTH 5   
#define FILTER_SIZE 10  

uint8_t dataBuffer[DATA_LENGTH];
uint8_t dataIndex = 0;  

// Initialize components
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, SENSOR_CS_PIN);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_PIN1, MOTOR_PWM_PIN2, MOTOR_PWM_PIN3, MOTOR_ENABLE_PIN);


float startAngle_motor = 0;
float target_velocity = 0.0;
float previous_motor_angle = 0.0;
float convert_rpm;

String inputString = "";
bool stringComplete = false;
uint16_t count_time_out = 0;
uint16_t count_target_velocity = 0; 

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t correctedAngle_motor;
FLOATUNION_t motor_speed_rpm;

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (dataIndex < DATA_LENGTH) {
      dataBuffer[dataIndex++] = inChar;
    }
    if (inChar == '\n') {
      stringComplete = true;
      break; 
    }
  }

  if (stringComplete) {
    if (dataIndex == DATA_LENGTH) { 
      if (dataBuffer[0] == 'T') {
        float receivedFloat;
        uint32_t floatBits = (uint32_t)dataBuffer[4] << 24 | ((uint32_t)dataBuffer[3] << 16) | ((uint32_t)dataBuffer[2] << 8) | ((uint32_t)dataBuffer[1]);
        memcpy(&receivedFloat, &floatBits, sizeof(receivedFloat));
        
        if (receivedFloat > 20.0)
          target_velocity = 20.0;
        else if (receivedFloat < -20.0)
          target_velocity = -20.0;
        else
          target_velocity = receivedFloat;
        
        count_time_out = 0;
        count_target_velocity = 0;
      }
    }
  }
  
  dataIndex = 0;
  stringComplete = false;
  memset(dataBuffer, 0, DATA_LENGTH); 
}

// Initialize start angles for motor
void initializeStartAngles() {
  sensor.update();
  startAngle_motor = sensor.getAngle();
  previous_motor_angle = startAngle_motor;
}

void setup() {
  Serial.begin(115200);

  sensor.init();
  motor.linkSensor(&sensor);

  convert_rpm = 60.0 / (2 * PI); 

  // Initialize motor driver
  driver.voltage_power_supply = VOLTAGE_POWER;
  driver.voltage_limit = VOLTAGE_LIMIT;
  driver.init(); 

  // Link the motor and the driver
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 18;

  motor.controller = MotionControlType::torque;
  motor.init(); 
  motor.initFOC();

  initializeStartAngles();
}

uint8_t send_data = 0;

void loop() {

  motor.loopFOC();

  correctedAngle_motor.number = sensor.getAngle() - startAngle_motor;

  motor_speed_rpm.number = sensor.getVelocity(); 

  if (send_data > 20) { 
    Serial.write('A'); 
    for (uint8_t i = 0; i < 4; i++) {
      Serial.write(correctedAngle_motor.bytes[i]);
    }
    for (uint8_t i = 0; i < 4; i++) {
      Serial.write(motor_speed_rpm.bytes[i]);
    }
    Serial.print('\n'); 
    send_data = 0; 
  }
  send_data = send_data + 1; 

  motor.move(target_velocity);

  if (abs(target_velocity) > 0.0 && stringComplete == false && count_time_out > 1000) {
    target_velocity = 0.0;
    count_time_out = 1001;
  }

  count_time_out = count_time_out + 1;
}
