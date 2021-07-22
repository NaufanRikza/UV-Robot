#include <Arduino.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_camera.h"
#include "SPIFFS.h"
#include "WiFi.h"
#include <ArduinoWebsockets.h>
#include <AsyncTCP.h>
#include <AsyncWebSocket.h>
#include "speedgraph.h"

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#define DIR_RIGHT_1 15
#define DIR_RIGHT_2 13
#define DIR_LEFT_1  2
#define DIR_LEFT_2 4
#define PWM_RIGHT 12
#define PWM_LEFT 14
#define UV_LAMP 3

#define FORWARD 0
#define BACKWARD 1
#define FORWARD_RIGHT 2
#define FORWARD_LEFT 3
#define BACKWARD_LEFT 4
#define BACKWARD_RIGHT 5
#define ROT_RIGHT 6
#define ROT_LEFT 7
#define NEUTRAL 8

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define MAX_SPEED_SAMPLING 1
#define MIN_SPEED_SAMPLING 1

#define NOT_USED 0
#define SPEEDUP 1
#define SPEEDDOWN 2

#define MAX_SAMPLING_TIME 1500

#define INTERVAL 5000
#define PWM_MIN 15

class Robot{
private:
void WritePWM(uint8_t dutycycle, uint8_t motor);
void RotateMotor(uint8_t motor, uint8_t speedvalue, uint8_t direc);
void RampSpeedUp(uint8_t moveflag);
void RampSpeedStop(uint8_t moveflag);
void NoRamp(uint8_t moveflag);
void RampUpWithMicros(uint8_t moveflag);
void RampStopWithMicros(uint8_t moveflag);
void LinearUpAcc(uint8_t moveflag);
void LinearDownAcc(uint8_t moveflag);
void Speedup_exp(uint8_t moveflag);
void Speeddown_exp(uint8_t moveflag);
void Set_Sigmoid_risetime(uint16_t time);
void Set_sigmoid_Const(float Const);
mcpwm_config_t PWM_Config;
camera_config_t config;
sensor_t * s;

public:
int16_t posX;
int16_t posY;
uint8_t speed = 30;
uint8_t relay;
uint8_t maxVelocityBefore = 0;
uint8_t maxVelocity = 0;
float maxVelocitysigmoid = 0;
uint8_t minVelocity = 20;
uint8_t rotVelocity = 60;
double minvelocitysigmoid = 0;
uint8_t noVelocity = 0;
uint8_t speednow = 0;
bool RAMPDOWN = false;
volatile uint8_t ramptiming = NOT_USED;
const int maxRange = 35;
const int minRange = -35;
hw_timer_t * timer2 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int FOWARD_FLAG = 0, INT_MOVEFLAG;
volatile int MOVE_FLAG = NEUTRAL;
int PREV_MOVE=NEUTRAL;
double RAMP_SPEED = 0, RAMP_SPEED_MIN = 0;
unsigned long timenow=0, timebefore=0;
std::vector <double> SigmoidVal;
std::vector <double> SigmoidPWM;
volatile uint16_t Maxsigmoidindex;

public:
camera_fb_t * fb = NULL;
void begin();
void MoveNoRamp(int8_t x, int8_t y);
void UVLamp(uint8_t state);
void Move(int8_t x, int8_t y);
void MoveLinear(int8_t x, int8_t y);
float CalculatePWMSigmoidConst(double maxspeed);
double DrawSigmoidPWM(double sigmoidval);
double DrawSigmoidPWMDown(double sigmoidval);
uint8_t LinearFunctionCalc(uint16_t time);
uint8_t LinearFunctionCalcdown(uint16_t time);
};