#include <Arduino.h>
#include "Robot.h"

using namespace websockets;
Robot UVRobot;
WebsocketsServer server;
AsyncWebServer webserver(80);
TaskHandle_t MovementControl;
bool calculate = false;
bool sigmoid = false;
volatile uint8_t pinstate = LOW;

void IRAM_ATTR SigmoidWrite()
{
  portENTER_CRITICAL_ISR(&UVRobot.timerMux);
  if(sigmoid_index <= UVRobot.Maxsigmoidindex)
  {
    sigmoid_index++;
  }
  else
  {
    sigmoid_index = UVRobot.Maxsigmoidindex;
  }

  switch(UVRobot.ramptiming)
  {
    case SPEEDUP:
    {
      UVRobot.RAMP_SPEED = UVRobot.DrawSigmoidPWM(UVRobot.SigmoidVal[sigmoid_index]);
      if((UVRobot.MOVE_FLAG == FORWARD_RIGHT || UVRobot.MOVE_FLAG == FORWARD_LEFT 
        ||UVRobot.MOVE_FLAG == BACKWARD_RIGHT || UVRobot.MOVE_FLAG == BACKWARD_LEFT) 
        && (UVRobot.RAMP_SPEED < UVRobot.minVelocity))
      {
        UVRobot.RAMP_SPEED_MIN = UVRobot.RAMP_SPEED;
      }      
    }break;

    case SPEEDDOWN:
    {
      UVRobot.RAMP_SPEED= UVRobot.DrawSigmoidPWMDown(UVRobot.SigmoidVal[UVRobot.Maxsigmoidindex - sigmoid_index]);
      if((UVRobot.MOVE_FLAG == FORWARD_RIGHT || UVRobot.MOVE_FLAG == FORWARD_LEFT 
        ||UVRobot.MOVE_FLAG == BACKWARD_RIGHT || UVRobot.MOVE_FLAG == BACKWARD_LEFT) 
        && (UVRobot.RAMP_SPEED > UVRobot.minVelocity))
      {
        UVRobot.RAMP_SPEED_MIN = UVRobot.RAMP_SPEED;
      }
    }break;
  }  
  portEXIT_CRITICAL_ISR(&UVRobot.timerMux);  
}

void IRAM_ATTR time()
{
  if(elapsedtime < MAX_SAMPLING_TIME)
  {
    elapsedtime++;
  }
  else
  {
    elapsedtime = MAX_SAMPLING_TIME;
  }

  switch(UVRobot.ramptiming)
  {
    case SPEEDUP:
    {
      UVRobot.RAMP_SPEED = UVRobot.LinearFunctionCalc(elapsedtime);
      if((UVRobot.MOVE_FLAG == FORWARD_RIGHT || UVRobot.MOVE_FLAG == FORWARD_LEFT 
        ||UVRobot.MOVE_FLAG == BACKWARD_RIGHT || UVRobot.MOVE_FLAG == BACKWARD_LEFT) 
        && (UVRobot.RAMP_SPEED < UVRobot.minVelocity))
      {
        UVRobot.RAMP_SPEED_MIN = UVRobot.RAMP_SPEED;
      }      
    }break;

    case SPEEDDOWN:
    {
      UVRobot.RAMP_SPEED = UVRobot.LinearFunctionCalcdown(elapsedtime);
      if((UVRobot.MOVE_FLAG == FORWARD_RIGHT || UVRobot.MOVE_FLAG == FORWARD_LEFT 
        ||UVRobot.MOVE_FLAG == BACKWARD_RIGHT || UVRobot.MOVE_FLAG == BACKWARD_LEFT) 
        && (UVRobot.RAMP_SPEED > UVRobot.minVelocity))
      {
        UVRobot.RAMP_SPEED_MIN = UVRobot.RAMP_SPEED;
      }
    }break;
  }  
  portEXIT_CRITICAL_ISR(&UVRobot.timerMux);
}

void loop2(void * parameter)
{
  for(;;)
  {
    UVRobot.UVLamp(UVRobot.relay);
    // UVRobot.Move(UVRobot.posX, UVRobot.posY);
    // UVRobot.MoveNoRamp(UVRobot.posX, UVRobot.posY);
    UVRobot.MoveLinear(UVRobot.posX, UVRobot.posY);
    // Serial.println(sigmoid_index);
    vTaskDelay(1);
  }
}

void timer2init()
{
  UVRobot.timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(UVRobot.timer2, &SigmoidWrite, true);
  timerAlarmWrite(UVRobot.timer2, 10000, true);
  timerAlarmEnable(UVRobot.timer2);
  timerStop(UVRobot.timer2);
  timerWrite(UVRobot.timer2, 0);
}

void handle_message(WebsocketsMessage msg) {
  String dataJoystick = msg.data();
  // Serial.println(dataJoystick);
  int index = dataJoystick.indexOf(',');
  int index2 = dataJoystick.indexOf(',',index+1);
  int index3 = dataJoystick.indexOf(',',index2+1);
  int index4 = dataJoystick.indexOf(',',index3+1);
  
  String posX_S = dataJoystick.substring(0, index);
  String posY_S = dataJoystick.substring(index + 1, index2);
  String relay_S = dataJoystick.substring(index2 + 1, index3);
  String speed_S = dataJoystick.substring(index3 + 1, index4);

  UVRobot.posX = posX_S.toInt();
  UVRobot.posY = posY_S.toInt();
  UVRobot.relay = relay_S.toInt();
  UVRobot.speed = speed_S.toInt();

  UVRobot.maxVelocity =  map(UVRobot.speed,1,100,30,100);
  if((UVRobot.maxVelocity != UVRobot.maxVelocityBefore))
  {
    PWMsigmoidconst = UVRobot.CalculatePWMSigmoidConst(UVRobot.maxVelocity);
    UVRobot.maxVelocitysigmoid = UVRobot.DrawSigmoidPWM(UVRobot.SigmoidVal[UVRobot.Maxsigmoidindex]);
    UVRobot.minvelocitysigmoid = UVRobot.DrawSigmoidPWM(UVRobot.SigmoidVal[0]);
    Serial.println(PWMsigmoidconst, 3);
  }
  UVRobot.maxVelocityBefore = UVRobot.maxVelocity;
}

void setup() {
//  Serial.begin(115200);
 UVRobot.begin();

 webserver.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });
 webserver.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });
 webserver.on("/joy.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/joy.js", "text/javascript");
  });
 webserver.on("/initialimage", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/initial.jpeg", "image/jpeg");
  });
  
  webserver.begin();
  server.listen(82);
  xTaskCreatePinnedToCore(loop2,"MovementControl",10000,NULL,0,&MovementControl,0);
  timer2init();
}

void loop() {
  auto client = server.accept();
  client.onMessage(handle_message);
  while (client.available()) 
  { 
    client.poll();
    UVRobot.fb = esp_camera_fb_get();
    client.sendBinary((const char *)UVRobot.fb->buf, UVRobot.fb->len);
    esp_camera_fb_return(UVRobot.fb);
    UVRobot.fb = NULL;
  }
}