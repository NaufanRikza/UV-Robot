#include "Robot.h"

#define freq 20000

void Robot::UVLamp(uint8_t state)
{
  digitalWrite(UV_LAMP, state);
}

void Robot::Set_Sigmoid_risetime(uint16_t time)
{
  switch (time)
  {
    case 300:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_300),std::end(sigmoid_300));
    }break;

    case 500:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_500_new),std::end(sigmoid_500_new));
    }break;

    case 700:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_700_new),std::end(sigmoid_700_new));
    }break;

    case 900:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_900_new),std::end(sigmoid_900_new));
    }break;

    case 1200:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_1200),std::end(sigmoid_1200));
    }break;

    case 1500:
    {
      SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_1500),std::end(sigmoid_1500));
    }break;
  }

  SigmoidVal.shrink_to_fit();
  Maxsigmoidindex = SigmoidVal.size() - 1;
  Serial.print("Index = ");
  Serial.println(Maxsigmoidindex);

  for(int i=0;i<=1;i++)
  {
    if(i == 0)
    {
      for(int j = 0;j<=Maxsigmoidindex;j++)
      {
        Serial.println(SigmoidVal[j],4);
      }
    }
    else if(i == 1)
    {
      Serial.println("-------------");
      for(int j = 0;j<=Maxsigmoidindex;j++)
      {
        Serial.println(SigmoidVal[Maxsigmoidindex - j],4);
      }
    }
  }
}

void Robot::Set_sigmoid_Const(float Const)
{
  if(Const == 0.5)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_05),std::end(sigmoid_05));
  }
  else if(Const == 1)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_1),std::end(sigmoid_1));
  }
  else if(Const == 1.25)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_125),std::end(sigmoid_125));
  }
  else if(Const == 1.5)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_15),std::end(sigmoid_15));
  }
  else if(Const == 1.75)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_175),std::end(sigmoid_175));
  }
  else if(Const == 2)
  {
    SigmoidVal.insert(SigmoidVal.begin(),std::begin(sigmoid_2),std::end(sigmoid_2));
  }

  SigmoidVal.shrink_to_fit();
  Maxsigmoidindex = SigmoidVal.size() - 1;
}

double Robot::DrawSigmoidPWM(double sigmoidval)
{
  return ((sigmoidval + PWMsigmoidconst)/(1.0 + PWMsigmoidconst)) * maxVelocity;
}

float Robot::CalculatePWMSigmoidConst(double maxspeed)
{
  return (double)PWM_MIN/(maxspeed - PWM_MIN);
}

void Robot::begin()
{
  pinMode(DIR_RIGHT_1, OUTPUT);
  pinMode(DIR_RIGHT_2, OUTPUT);
  pinMode(DIR_LEFT_1, OUTPUT);
  pinMode(DIR_LEFT_2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  // pinMode(UV_LAMP, OUTPUT);

  // Set_Sigmoid_risetime(500);S
  Set_sigmoid_Const(1.25);

  PWM_Config.frequency = freq; //frequency,
  PWM_Config.cmpr_a = 0;       //duty cycle of PWMxA = 0
  PWM_Config.cmpr_b = 0;       //duty cycle of PWMxb = 0
  PWM_Config.counter_mode = MCPWM_UP_COUNTER;
  PWM_Config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &PWM_Config);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_RIGHT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_LEFT);

  // UVLamp(LOW);

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("TRILILI", "12345678");
  IPAddress myIP = WiFi.softAPIP();

  Serial.println(myIP); // You can get IP address assigned to ESP

}

void Robot::WritePWM(uint8_t dutycycle, uint8_t motor)
{
  if (dutycycle > 0 && dutycycle < 100)
  {
    if (motor == RIGHT_MOTOR)
    {
      //       mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutycycle);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
      //       mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dutycycle);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
  }
  else if (dutycycle == 0)
  {
    if (motor == RIGHT_MOTOR)
    {
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    else
    {
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
  }
  else if (dutycycle == 100)
  {
    if (motor == RIGHT_MOTOR)
    {
      mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    else
    {
      mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
    }
  }
}

void Robot::RotateMotor(uint8_t motor, uint8_t speedvalue, uint8_t direc)
{
  switch (motor)
  {
  case RIGHT_MOTOR:
  {
    if (direc == FORWARD)
    {
      digitalWrite(DIR_RIGHT_1, HIGH);
      digitalWrite(DIR_RIGHT_2, LOW);
    }
    else if(direc == BACKWARD)
    {
      digitalWrite(DIR_RIGHT_1, LOW);
      digitalWrite(DIR_RIGHT_2, HIGH);
    }
    else if(direc == NEUTRAL)
    {
      digitalWrite(DIR_RIGHT_1, LOW);
      digitalWrite(DIR_RIGHT_2, LOW);
    }
    WritePWM(speedvalue, motor);
  }
  break;

  case LEFT_MOTOR:
  {
    if (direc == FORWARD)
    {
      digitalWrite(DIR_LEFT_1, HIGH);
      digitalWrite(DIR_LEFT_2, LOW);
    }
    else if(direc == BACKWARD)
    {
      digitalWrite(DIR_LEFT_1, LOW);
      digitalWrite(DIR_LEFT_2, HIGH);
    }
    else if(direc == NEUTRAL)
    {
      digitalWrite(DIR_LEFT_1, LOW);
      digitalWrite(DIR_LEFT_2, LOW);
    }
    WritePWM(speedvalue, motor);
  }
  break;
  }
}

uint8_t Robot::LinearFunctionCalc(uint16_t time)
{
  return (maxVelocity * time) / MAX_SAMPLING_TIME;
}

uint8_t Robot::LinearFunctionCalcdown(uint16_t time)
{
  return (((MAX_SAMPLING_TIME*maxVelocity) - (maxVelocity) * time)) / MAX_SAMPLING_TIME; 
}

void Robot::RampSpeedUp(uint8_t moveflag)
{
  switch (moveflag)
  {
  case FORWARD:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        // Serial.println(RAMP_SPEED);
        RAMP_SPEED += MAX_SPEED_SAMPLING;
        if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
    }
    else
    {
      // Serial.println(RAMP_SPEED);
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case BACKWARD:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        RAMP_SPEED += MAX_SPEED_SAMPLING;
        if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;

  case FORWARD_LEFT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD);
        }
        else
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
    }
  }
  break;

  case FORWARD_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case BACKWARD_LEFT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
        else
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
    }
  }
  break;

  case BACKWARD_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;

  case ROT_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        RAMP_SPEED += MAX_SPEED_SAMPLING;
        if(RAMP_SPEED >= maxVelocity)RAMP_SPEED = maxVelocity;

        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case ROT_LEFT:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        RAMP_SPEED += MAX_SPEED_SAMPLING;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      if(RAMP_SPEED >= maxVelocity)RAMP_SPEED = maxVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;
  }
}

void Robot::RampSpeedStop(uint8_t moveflag)
{
  switch (moveflag)
  {
  case FORWARD:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        Serial.println(RAMP_SPEED);
        RAMP_SPEED -= MAX_SPEED_SAMPLING;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      Serial.println(RAMP_SPEED);
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        RAMP_SPEED -= MAX_SPEED_SAMPLING;
        if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case FORWARD_LEFT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if(RAMP_SPEED > minVelocity)
        {
          RAMP_SPEED -=MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);          
        }
        else
        {
          RAMP_SPEED -=MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN -=MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD); 
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case FORWARD_RIGHT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN -= MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD_LEFT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED > minVelocity)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
        else
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN -= MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;      
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD_RIGHT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED > minVelocity)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          RAMP_SPEED_MIN -= MIN_SPEED_SAMPLING;
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;      
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case ROT_RIGHT:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        RAMP_SPEED -= MAX_SPEED_SAMPLING;

        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case ROT_LEFT:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        RAMP_SPEED -= MAX_SPEED_SAMPLING;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;
  }  
}

void Robot::RampUpWithMicros(uint8_t moveflag)
{
  switch (moveflag)
  {
  case FORWARD:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        timenow = micros();
        // Serial.println(RAMP_SPEED);
        if(((timenow - timebefore)) >= INTERVAL)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
    }
    else
    {
      // Serial.println(RAMP_SPEED);
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case BACKWARD:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        timenow = micros();
        if(((timenow - timebefore)) >= INTERVAL)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;

  case FORWARD_LEFT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
    }
  }
  break;

  case FORWARD_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case BACKWARD_LEFT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
    }
  }
  break;

  case BACKWARD_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity && RAMP_SPEED_MIN < minVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN += MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED += MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      if(RAMP_SPEED > maxVelocity)RAMP_SPEED = maxVelocity;
      if(RAMP_SPEED_MIN > minVelocity)RAMP_SPEED_MIN = minVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;

  case ROT_RIGHT:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        timenow = micros();
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        if(RAMP_SPEED >= maxVelocity)RAMP_SPEED = maxVelocity;

        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
    }
  }
  break;

  case ROT_LEFT:
  {
    if (RAMP_SPEED < maxVelocity)
    {
      while (RAMP_SPEED < maxVelocity)
      {
        timenow = micros();
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED += MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      if(RAMP_SPEED >= maxVelocity)RAMP_SPEED = maxVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
      RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
    }
  }
  break;
  }
}

void Robot::RampStopWithMicros(uint8_t moveflag)
{
  switch (moveflag)
  {
  case FORWARD:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        timenow = micros();
        Serial.println(RAMP_SPEED);
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      Serial.println(RAMP_SPEED);
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        timenow = micros();
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case FORWARD_LEFT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if(RAMP_SPEED > minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);          
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN -=MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD); 
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case FORWARD_RIGHT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED_MIN < minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN -=MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD_LEFT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED > minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN -=MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;      
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case BACKWARD_RIGHT:
  {
    if (RAMP_SPEED > noVelocity && RAMP_SPEED_MIN > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        if (RAMP_SPEED > minVelocity)
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          timenow = micros();
          if((timenow - timebefore) >= INTERVAL)
          {
            RAMP_SPEED -= MAX_SPEED_SAMPLING;
            RAMP_SPEED_MIN -=MIN_SPEED_SAMPLING;
            timebefore = timenow;
          }
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
      if(RAMP_SPEED_MIN < noVelocity)RAMP_SPEED_MIN = noVelocity;      
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case ROT_RIGHT:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        timenow = micros();
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }

        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;

  case ROT_LEFT:
  {
    if (RAMP_SPEED > noVelocity)
    {
      while (RAMP_SPEED > noVelocity)
      {
        timenow = micros();
        if((timenow - timebefore) >= INTERVAL)
        {
          RAMP_SPEED -= MAX_SPEED_SAMPLING;
          timebefore = timenow;
        }
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      if(RAMP_SPEED < noVelocity)RAMP_SPEED = noVelocity;
    }
    else
    {
      RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
      RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    }
  }
  break;
  }    
}

void Robot::NoRamp(uint8_t moveflag)
{
  if(moveflag == NEUTRAL)
  {
    speednow = noVelocity;
  }
  else
  {
    speednow = maxVelocity;
  }

  // Serial.println(speednow);
  
  switch (moveflag)
  {
  case FORWARD:
  {
    RotateMotor(RIGHT_MOTOR, speednow, FORWARD);
    RotateMotor(LEFT_MOTOR, speednow, FORWARD);
  }
  break;

  case BACKWARD:
  {
    RotateMotor(RIGHT_MOTOR, speednow, BACKWARD);
    RotateMotor(LEFT_MOTOR, speednow, BACKWARD);
  }
  break;

  case FORWARD_LEFT:
  {
    RotateMotor(RIGHT_MOTOR, speednow, FORWARD);
    RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);    
  }
  break;

  case FORWARD_RIGHT:
  {
    RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
    RotateMotor(LEFT_MOTOR, speednow, FORWARD);
  }
  break;

  case BACKWARD_RIGHT:
  {
    RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
    RotateMotor(LEFT_MOTOR, speednow, BACKWARD);
  }
  break;

  case BACKWARD_LEFT:
  {
    RotateMotor(RIGHT_MOTOR, speednow, BACKWARD);
    RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
  }
  break;

  case ROT_RIGHT:
  {  
    RotateMotor(RIGHT_MOTOR, speednow, BACKWARD);
    RotateMotor(LEFT_MOTOR, speednow, FORWARD);   
  }
  break;

  case ROT_LEFT:
  {
    RotateMotor(RIGHT_MOTOR, speednow, FORWARD);
    RotateMotor(LEFT_MOTOR, speednow, BACKWARD);
  }
  break;

  case NEUTRAL:
  {
    RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
    RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
  }
  break;
  }
}

void Robot::LinearUpAcc(uint8_t moveflag)
{
  // Serial.println("MASUK");
  if (!timerStarted(timer2) && (RAMP_SPEED == noVelocity))
  {
    Serial.println("|");
    ramptiming = SPEEDUP;
    timerStart(timer2);
  }
  else
  {
    if ((RAMP_SPEED >= maxVelocity) && (timerStarted(timer2)))
    {
      RAMP_SPEED = maxVelocity;
      ramptiming = NOT_USED;
      timerStop(timer2);
      timerWrite(timer2, 0);
      elapsedtime = 0;
      sigmoid_index = 0;
      RAMPDOWN = true;
    }
    else if (ramptiming == SPEEDUP)
    {
      switch (moveflag)
      {
      case FORWARD:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      break;

      case FORWARD_LEFT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
        }
      }
      break;

      case FORWARD_RIGHT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      break;

      case BACKWARD:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      break;

      case BACKWARD_LEFT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
      }
      break;

      case BACKWARD_RIGHT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      break;

      case ROT_RIGHT:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      break;

      case ROT_LEFT:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      break;
      }
    }
    else if (ramptiming == NOT_USED && RAMP_SPEED >= maxVelocity)
    {
      switch (moveflag)
      {
      case FORWARD:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
      }
      break;

      case FORWARD_LEFT:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
        RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
      }
      break;

      case FORWARD_RIGHT:
      {
        RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
      }
      break;

      case BACKWARD:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
      }
      break;

      case BACKWARD_LEFT:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
        RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
      }
      break;

      case BACKWARD_RIGHT:
      {
        RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
      }
      break;

      case ROT_LEFT:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, FORWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, BACKWARD);
      }
      break;

      case ROT_RIGHT:
      {
        RotateMotor(RIGHT_MOTOR, maxVelocity, BACKWARD);
        RotateMotor(LEFT_MOTOR, maxVelocity, FORWARD);
      }
      break;
      }
    }
  }
}

void Robot::LinearDownAcc(uint8_t moveflag)
{
  if ((!timerStarted(timer2)) && (RAMP_SPEED == maxVelocity))
  {
    ramptiming = SPEEDDOWN;
    timerStart(timer2);
  }
  else
  {
    if ((RAMP_SPEED <= noVelocity) && (timerStarted(timer2)))
    {
      RAMP_SPEED = noVelocity;
      ramptiming = NOT_USED;
      timerStop(timer2);
      timerWrite(timer2, 0);
      elapsedtime = 0;
      sigmoid_index = 0;
      RAMPDOWN = false;
    }
    else if (ramptiming == SPEEDDOWN)
    {
      switch (moveflag)
      {
      case FORWARD:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      break;

      case FORWARD_LEFT:
      {
        if (RAMP_SPEED <= minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, FORWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, FORWARD);
        }
      }
      break;

      case FORWARD_RIGHT:
      {
        if (RAMP_SPEED <= minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, minVelocity, FORWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
        }
      }
      break;

      case BACKWARD:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      break;

      case BACKWARD_LEFT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
          RotateMotor(LEFT_MOTOR, minVelocity, BACKWARD);
        }
      }
      break;

      case BACKWARD_RIGHT:
      {
        if (RAMP_SPEED < minVelocity)
        {
          RotateMotor(RIGHT_MOTOR, RAMP_SPEED_MIN, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
        else
        {
          RotateMotor(RIGHT_MOTOR, minVelocity, BACKWARD);
          RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
        }
      }
      break;

      case ROT_RIGHT:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, BACKWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, FORWARD);
      }
      break;

      case ROT_LEFT:
      {
        RotateMotor(RIGHT_MOTOR, RAMP_SPEED, FORWARD);
        RotateMotor(LEFT_MOTOR, RAMP_SPEED, BACKWARD);
      }
      break;
      }
    }
    else if (ramptiming == NOT_USED)
    {
      if (RAMP_SPEED <= noVelocity)
      {
        RAMP_SPEED = noVelocity;
        RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
        RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
      }
    }
  }
}

void Robot::MoveNoRamp(int8_t x, int8_t y)
{
  if (y >= maxRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = FORWARD;
    // Serial.println("MAJU");
  }
  // Move forward right
  else if (x >= maxRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_RIGHT;
    // Serial.println("MAJU KANAN");
  }
  // Move forward left
  else if (x <= minRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_LEFT;
    // Serial.println("MAJU KIRI");
  }
  // Neutral
  else if ((y < maxRange && y > minRange) && (x < maxRange && x > minRange))
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("DIAM");
  }
  // Move back
  else if (y <= minRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = BACKWARD;
    // Serial.println("MUNDUR");
  }
  // Move back and right
  else if (y < minRange && x >= maxRange)
  {
    MOVE_FLAG = BACKWARD_RIGHT;
    // Serial.println("MUNDUR KANAN");
  }
  // Move back and left
  else if (y < minRange && x <= minRange)
  {
    MOVE_FLAG = BACKWARD_LEFT;
    // Serial.println("MUNDUR KIRI");
  }
  else if ((y >= minRange && y <= maxRange) && x >= maxRange)
  {
    MOVE_FLAG = ROT_RIGHT;
    // Serial.println("PUTAR KANAN");
  }
  else if ((y >= minRange && y <= maxRange) && x <= maxRange)
  {
    MOVE_FLAG = ROT_LEFT;
    // Serial.println("PUTAR KIRI");
  }
  else
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("NETRAL");
  }
  NoRamp(MOVE_FLAG);
}


void Robot::Move(int8_t x, int8_t y)
{
   if (y >= maxRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = FORWARD;
    // Serial.println("MAJU");
  }
  // Move forward right
  else if (x >= maxRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_RIGHT;
    // Serial.println("MAJU KANAN");
  }
  // Move forward left
  else if (x <= minRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_LEFT;
    // Serial.println("MAJU KIRI");
  }
  // Neutral
  else if ((y < maxRange && y > minRange) && (x < maxRange && x > minRange))
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("DIAM");
  }
  // Move back
  else if (y <= minRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = BACKWARD;
    // Serial.println("MUNDUR");
  }
  // Move back and right
  else if (y < minRange && x >= maxRange)
  {
    MOVE_FLAG = BACKWARD_RIGHT;
    // Serial.println("MUNDUR KANAN");
  }
  // Move back and left
  else if (y < minRange && x <= minRange)
  {
    MOVE_FLAG = BACKWARD_LEFT;
    // Serial.println("MUNDUR KIRI");
  }
  else if ((y >= minRange && y <= maxRange) && x >= maxRange)
  {
    MOVE_FLAG = ROT_RIGHT;
    // Serial.println("PUTAR KANAN");
  }
  else if ((y >= minRange && y <= maxRange) && x <= maxRange)
  {
    MOVE_FLAG = ROT_LEFT;
    // Serial.println("PUTAR KIRI");
  }
  else
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("NETRAL");
  }

  if(MOVE_FLAG != NEUTRAL)
  {
    // Serial.println("SPEED UP");
    // RampSpeedUp(MOVE_FLAG);
    RampUpWithMicros(MOVE_FLAG);
  }
  else if(MOVE_FLAG == NEUTRAL && PREV_MOVE != NEUTRAL)
  {
    // Serial.println("SPEED DOWN");
    // RampSpeedStop(PREV_MOVE);
    RampStopWithMicros(PREV_MOVE);
  }
  else if(MOVE_FLAG == NEUTRAL && PREV_MOVE == NEUTRAL)
  {
    // Serial.println(RAMP_SPEED);
    RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
    RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
  }
  // Serial.println(RAMP_SPEED);
  PREV_MOVE = MOVE_FLAG;
}

void Robot::MoveLinear(int8_t x, int8_t y)
{
  if (y >= maxRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = FORWARD;
    // Serial.println("MAJU");
  }
  // Move forward right
  else if (x >= maxRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_RIGHT;
    // Serial.println("MAJU KANAN");
  }
  // Move forward left
  else if (x <= minRange && y >= maxRange)
  {
    MOVE_FLAG = FORWARD_LEFT;
    // Serial.println("MAJU KIRI");
  }
  // Neutral
  else if ((y < maxRange && y > minRange) && (x < maxRange && x > minRange))
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("DIAM");
  }
  // Move back
  else if (y <= minRange && (x >= minRange && x <= maxRange))
  {
    MOVE_FLAG = BACKWARD;
    // Serial.println("MUNDUR");
  }
  // Move back and right
  else if (y < minRange && x >= maxRange)
  {
    MOVE_FLAG = BACKWARD_RIGHT;
    // Serial.println("MUNDUR KANAN");
  }
  // Move back and left
  else if (y < minRange && x <= minRange)
  {
    MOVE_FLAG = BACKWARD_LEFT;
    // Serial.println("MUNDUR KIRI");
  }
  else if ((y >= minRange && y <= maxRange) && x >= maxRange)
  {
    MOVE_FLAG = ROT_RIGHT;
    // Serial.println("PUTAR KANAN");
  }
  else if ((y >= minRange && y <= maxRange) && x <= maxRange)
  {
    MOVE_FLAG = ROT_LEFT;
    // Serial.println("PUTAR KIRI");
  }
  else
  {
    MOVE_FLAG = NEUTRAL;
    // Serial.println("NETRAL");
  }

  if (MOVE_FLAG != NEUTRAL)
  {
    LinearUpAcc(MOVE_FLAG);
  }
  else if (MOVE_FLAG == NEUTRAL && PREV_MOVE != NEUTRAL)
  {
    if (RAMPDOWN)
    {
      INT_MOVEFLAG = PREV_MOVE;
    }
  }
  else if (MOVE_FLAG == NEUTRAL && PREV_MOVE == NEUTRAL && RAMPDOWN)
  {
    // Serial.println(RAMP_SPEED);
    LinearDownAcc(INT_MOVEFLAG);
  }
  else
  {
    RotateMotor(RIGHT_MOTOR, noVelocity, NEUTRAL);
    RotateMotor(LEFT_MOTOR, noVelocity, NEUTRAL);
    elapsedtime = 0;
    sigmoid_index = 0;
    // timerStop(timer2);
    timerWrite(timer2, 0);
  }
  // Serial.println(RAMP_SPEED);
  PREV_MOVE = MOVE_FLAG;
}