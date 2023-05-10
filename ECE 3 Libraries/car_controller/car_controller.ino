#include <ECE3.h>

const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const float k_p = 0.042;
const float k_d = 0.37;
const int base_speed = 175;

int16_t current_error = 0; //current weighted error value
int16_t previous_error = 0; //previous weighted error value
int16_t delta_error = 0;
uint16_t sensorValues[8];
uint16_t minSensorValues[8] = {739, 648, 597, 597, 544, 583, 620, 714};
uint16_t maxSensorValues[8] = {1761, 1774, 1753, 1184, 1186, 983, 1826, 1786};
const int16_t eight_weight_values[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

int16_t correction_signal;

int crosspiece_counter = 0;
bool turned = false;
bool crosspiece_last_read = false;
int16_t reads_since = 0;

void setup() {
  ECE3_Init();
  
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);

  digitalWrite(left_nslp_pin, HIGH);   // Turns on/off motor
  digitalWrite(right_nslp_pin, HIGH); 
  
  delay(2000); // Wait 2 seconds before starting

  ECE3_read_IR(sensorValues);

  for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] < minSensorValues[i])
      minSensorValues[i] = sensorValues[i];  
    else if (sensorValues[i] - minSensorValues[i] > maxSensorValues[i])
      maxSensorValues[i] = sensorValues[i] - minSensorValues[i];

   current_error = calc_error_func(sensorValues);
  }
}

void loop() {
  ECE3_read_IR(sensorValues);
  for (unsigned char i = 0; i < 8; i++) {
    if (sensorValues[i] < minSensorValues[i])
      minSensorValues[i] = sensorValues[i];  
    else if (sensorValues[i] - minSensorValues[i] > maxSensorValues[i])
      maxSensorValues[i] = sensorValues[i] - minSensorValues[i];
  }
  
  if (sensorValues[1] >= 2300 && sensorValues[6] >= 2300){ // Crosspiece checker, increments if a crosspiece is detected
    crosspiece_counter = crosspiece_counter + 1;
    crosspiece_last_read = true;
  }
  else if (crosspiece_last_read == true){ //If a crosspiece was detected last time, but not this time, then reset crosspiece_last_read and crosspiece_counter
    crosspiece_last_read = false;
    crosspiece_counter = 0;
  }

  if (crosspiece_counter == 2 && turned == false){ // If two crosspieces detected for the first time, execute donut maneuver
    crosspiece_counter = 0;
    reads_since = 0;
    analogWrite(left_pwm_pin, 255);
    analogWrite(right_pwm_pin, 255);
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,HIGH);
    delay(163);
    digitalWrite(left_dir_pin,LOW);
    digitalWrite(right_dir_pin,LOW);
    turned = true;
  }
  else if (crosspiece_counter == 2 && turned == true && reads_since > 250){ // If two crosspieces detected for the second time, stop wheels
      digitalWrite(left_nslp_pin, LOW); 
      digitalWrite(right_nslp_pin, LOW); 
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
  }
  else { // If no crosspieces detected, just run normally
    current_error = calc_error_func(sensorValues);
    delta_error = current_error - previous_error;
    
    // Use k_p and k_d to compute the correction signal
    correction_signal = current_error * k_p + delta_error * k_d;
    
    // Sends correction signal to motor
    analogWrite(left_pwm_pin, base_speed - correction_signal);
    analogWrite(right_pwm_pin, base_speed + correction_signal);
    reads_since = reads_since + 1;

  previous_error = current_error;
  }
}

int16_t calc_error_func(uint16_t sensor_val[]) {
  int error = 0;
  int calculation_var;

  for (unsigned char i = 0; i < 8; i++) {
    calculation_var = sensor_val[i] - minSensorValues[i];
    calculation_var = calculation_var * 1000;
    calculation_var = calculation_var / maxSensorValues[i];

    error += eight_weight_values[i]*calculation_var;
  }
  return error/8;
}
