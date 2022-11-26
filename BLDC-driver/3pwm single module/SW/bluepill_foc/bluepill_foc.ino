#include <Arduino.h>
//#include "stm32f1xx_hal.h"
#include <SimpleFOC.h>
#include "config.h"










// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(MotorPols);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA2, PB9);//(U,V,W,EN); pins 




InlineCurrentSense current_sense1 = InlineCurrentSense(0.001, 20, PA3, PA4);  // ( shunt res , gain of opamp , line 1 ADC pin , line 2 ADC pin)




// encoder instance
Encoder encoder1 = Encoder(PB6, PB7, 1024); // (A , B , pulse count / PPR  ) 
// channel A and B callbacks
void doA1() {
  encoder1.handleA();
}
void doB1() {
  encoder1.handleB();
}




//PIDController(float P, float I, float D, float ramp, float limit);
PIDController pid = PIDController{1, 1, 1, 1, 1}; 
// here just describe the PID instance. down there you can see I have to adjust each parameter one by one

// commander communication instance
Commander command = Commander(Serial);
void doMotor1(char* cmd) {
  command.motor(&motor1, cmd);
}


void onPid(char* cmd) {
  command.pid(&pid, cmd);
}


// A json based monitoring, also with sending J1 or 2 commands you can enable/disable the serial feedback
// i config the defualt as  VerboseMode::nothing 
void Send_info(char* cmd) {
  //if (cmd[0] == '0') Serial.println(analogRead(PA3));
  Serial.print ( "{\"encoder\":[");
  Serial.print(motor1.shaft_angle, 3);
  Serial.print(",");
  Serial.print(motor1.shaft_velocity, 3);

  Serial.println("]}");
if (cmd[0] == '1') command.verbose = VerboseMode::user_friendly;
if (cmd[0] == '2') command.verbose = VerboseMode::nothing;
  // VerboseMode::nothing        - display nothing - good for monitoring
  // VerboseMode::on_request     - display only on user request
  // VerboseMode::user_friendly  - display textual messages to the user (default)
  
};



void setup() {
  pinMode(LEDpin, OUTPUT); // 

  //  afio_cfg_debug_ports(AFIO_DEBUG_NONE);    //if you are not using stm32 official arduino firmware you might need to turn of the debug mode 


  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  delay(50);// wait for others to catchup 
  Serial.println("Dual foc setup begin ...");
  //  SimpleFOCDebug::enable();
  // SimpleFOCDebug::enable(&Serial);

  // VerboseMode::nothing        - display nothing - good for monitoring
  // VerboseMode::on_request     - display only on user request
  // VerboseMode::user_friendly  - display textual messages to the user (default)
  command.verbose = VerboseMode::nothing;


  digitalWrite(LEDpin, LOW);   // turn the LED on (HIGH is the voltage level)



  driver1.pwm_frequency = DriveFreq;


  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;




  // initialize encoder sensor hardware
  encoder1.init();
  encoder1.enableInterrupts(doA1, doB1);
 
  // link the motor to the sensor
  motor1.linkSensor(&encoder1);






  // driver config
  // power supply voltage [V]
  driver1.voltage_power_supply = inputVoltage;
  driver1.init();
  // link driver
  motor1.linkDriver(&driver1);
  // power supply voltage [V]


  // set control loop type to be used
  motor1.controller = MotionControlType::angle;  //velocity;  //torque

  motor1.voltage_sensor_align = 6;



  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  // motor1.LPF_velocity.Tf = 0.01;
  // motor1.LPF_velocity.Tf = 0.01;

  // contoller configuration based on the controll type
  motor1.PID_velocity.P = PID_V_P;
  motor1.PID_velocity.I = PID_V_I;
  motor1.PID_velocity.D = PID_V_D;
  motor1.PID_velocity.output_ramp = PID_V_ramp;

  motor1.LPF_velocity.Tf = PID_V_lpf;
  motor1.P_angle.P = PID_A_P;
  motor1.P_angle.I = PID_A_I;  // usually only P controller is enough
  motor1.P_angle.D = PID_A_D;  // usually only P controller is enough
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor1.P_angle.output_ramp = PID_A_ramp; // default 1e6 rad/s^2

  // angle low pass filtering
  // default 0 - disabled
  // use only for very noisy position sensors - try to avoid and keep the values very small
  motor1.LPF_angle.Tf = PID_A_lpf; // default 0



  // default voltage_power_supply
  motor1.voltage_limit = voltageLimit;     // the amoun of voltage that produce by SWM 
  motor1.current_limit = currentLimit; // Amps some how equal to max torque 










  // angle loop velocity limit
  motor1.velocity_limit = velocityLimit;



  // motor phase resistance [Ohms]
  motor1.phase_resistance = phaseResistance // Ohms - default not set
  // motor KV rating [rpm/V]
  //motor1.KV_rating = 100; // rpm/volt - default not set

  // comment out if not needed
  motor1.useMonitoring(Serial);

  motor1.monitor_downsample = 0; // disable monitor at first - optiona

  // initialise motor
  motor1.init();


  current_sense1.skip_align = true;


  int A1_Init = current_sense1.init();

  Serial.print(A1_Init);

  // init current sense
  if (A1_Init)  Serial.println("Current sense 1 init success!");
  else {
    Serial.println("Current sense 1 init failed!");
    //return;
  }
  // link the current sense to the motor
  motor1.linkCurrentSense(&current_sense1);

 




  Serial.println("Motor 1 FOC init start\n");

  // align encoder and start FOC
  motor1.initFOC();
  Serial.println("Motor 1 init finished\n");


  // set the inital target value
  motor1.target = 0;  //2



  command.decimal_places = 3; // default 3

  // subscribe motor to the commander
  command.add('M', doMotor1, "motor 1");

  command.add('C', onPid, "my pid");
  command.add('J', Send_info, "send P and V info ");
  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Double motor sketch ready."));

  _delay(100);
}


void loop() {

  // iterative setting FOC phase voltage
  motor1.loopFOC();


  // iterative function setting the outter loop target
  motor1.move();



  // real-time monitoring calls
//  motor1.monitor();


  // user communication
  command.run();
}
