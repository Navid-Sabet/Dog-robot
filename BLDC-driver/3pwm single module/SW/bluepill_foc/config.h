// motor parameters
#define MotorPols 20 // number of motor poles 
#define phaseResistance 1.6 // Ohms

#define PID_V_P      1.5   //1
#define PID_V_I       10    //1.5
#define PID_V_D    0.002 //0
#define PID_V_ramp   200
#define PID_V_lpf   0.05

#define PID_A_P       20
#define PID_A_I        0
#define PID_A_D        0
#define PID_A_ramp  1000
#define PID_A_lpf      0


// board and MCU configs 
#define LEDpin PC13
#define DriveFreq 10000
#define inputVoltage 24
#define currentLimit 9 // amps
#define voltageLimit 20 // volts
#define velocityLimit 10 // Rad/s
