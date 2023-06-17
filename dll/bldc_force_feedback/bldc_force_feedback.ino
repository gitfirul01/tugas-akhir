////////////////////// Dependencies ////////////////////// 
#include<Servo.h>

#define enc_pin 2
#define esc_pin 9
#define CLK_pin 3
#define DT_pin 4
#define ppr 14.0

////////////////////// Variables ////////////////////// 
// encoder
unsigned int rpm = 0;
unsigned long last_time = 0;
volatile byte pulse = 0;
static volatile unsigned long debounce = 0;


int counter = 0;
int current_state_CLK;
int last_state_CLK;
String current_dir = "";

// measured
float setpoint = 0.0;
float motor_pos = 0.0;
float prev_motor_pos = 0.0;

// pid-related variable
struct pidParam {
  float Kp = 0.5;
  float Ki = 3.0;
  float Kd = 0.0;
  float u = 0.0; //control signal
} _pid;

struct timeParam {
  float current_time = 0; //time in the moment of calculation
  float prev_time = 0; //for calculating delta t
  float delta_time = 0; //time difference
} _time;

struct errorParam {
  float error = 0; //error
  float prev_error = 0; //for calculating the derivative (edot)
  float integral_error = 0; //integral error
  float delta_error = 0; //derivative (de/dt)
} _error;

Servo ESC;


////////////////////// Setup Function ////////////////////// 
void setup() {
  Serial.begin(115200);

  pinMode(enc_pin, INPUT);
  pinMode(RotaryCLK, INPUT_PULLUP); //CLK
  pinMode(RotaryDT, INPUT_PULLUP); //DT

  ESC.attach(esc_pin, 1000, 2000);
  attachInterrupt(digitalPinToInterrupt(enc_pin), counter, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK_pin), update_sp, CHANGE);
}


////////////////////// Loop Function ////////////////////// 
void loop() {


  ESC.write(val);


}


////////////////////// Additional Function ////////////////////// 
void counter() {
  if ( digitalRead(enc_pin) && (micros() - debounce > 500) ) {
    debounce = micros();
    pulse++;
    Serial.println(pulse);
  }
}

void read_rpm() {
  if (millis() - last_time >= 1000) {
    noInterrupts();
    float rps = (pulse / ppr) / (1000 / (millis() - last_time));
    rpm = 60 * rps;
    Serial.println(rpm);

    last_time = millis();
    pulse = 0;
    interrupts();
  }
}

void update_sp() {
  current_state_CLK = digitalRead(CLK);

  if (current_state_CLK != last_state_CLK  && current_state_CLK == 1) {
    if (digitalRead(DT) != current_state_CLK) {
      counter --;
      current_dir = "CCW";
    } else {
      counter ++;
      current_dir = "CW";
    }
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.print(counter);
    Serial.print("\t");
    Serial.println(degree);
  }
  last_state_CLK = current_state_CLK;
}
