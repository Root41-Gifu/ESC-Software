char interval = 0;

int value = 20;
int integral = 0;

void ESC_Drive() {
  electricalAngle = getElectricalAngle();

  float neko = (float)integral / 1000.0;
  if (integral > 0) {
    pwmOutput((electricalAngle + 128 + 35) % 128, neko);
  } else {
    pwmOutput((electricalAngle + 128 - 35) % 128, neko * -1);
  }
  _electricalAngle = electricalAngle;

  HAL_Delay(10);
  interval++;
  interval %= 6;

  if (interval == 0) {
    velocity = mechanicalAngle - _mechanicalAngle;
    if (velocity <= -2000) {
      velocity += 4096;
    }
    if (velocity >= 2000) {
      velocity -= 4096;
    }
    _mechanicalAngle = mechanicalAngle;
    integral += value - velocity;
    integral = constrain(integral, -800, 800);
  }
}

int calibration(void) {
  changeFreq(125000);
  HAL_SYSTICK_Config(SystemCoreClock / (10000U / uwTickFreq));  // 2097152U

  while (true) {
    for (int i = 0; i < 128; i += 1) {
      mechanicalAngle = encoderRead();
      if (1500 <= mechanicalAngle && mechanicalAngle <= 1520) {
        goto END;
      }

      pwmOutput(i, 0.3);
      HAL_Delay(1);
    }
  }
END:
  pwmOutput(0, 0.2);
  HAL_Delay(300);
  pwmOutput(32, 0.1);
  HAL_Delay(100);
  pwmOutput(0, 0.2);
  HAL_Delay(300);
  pwmOutput(128 - 32, 0.1);
  HAL_Delay(100);
  pwmOutput(0, 0.3);
  HAL_Delay(200);

  mechanicalAngle = 0;
  for (int i = 0; i < 10; i++) {
    pwmOutput(0, 0.2);
    mechanicalAngle = (mechanicalAngle * i + encoderRead()) / (i + 1);
    HAL_Delay(500);
  }
  return 4096 - mechanicalAngle;
}

void ESC_initialize() {
  const int calibrationConstant[] = {2227, 2466, 2408, 2542};
  const int doremi[4] = {1047, 1175, 1319, 1396};
  int id = 0;
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 2; i++) {
      if (id == j) {
        reverse(1);
      }
      changeFreq(doremi[j]);
      pwmOutput(0, 0.1);
      HAL_Delay(70);
      reverse(0);
      HAL_Delay(30);
    }
  }
  for (int i = 2; i >= 0; i--) {
    reverse(1);
    changeFreq(doremi[i]);
    pwmOutput(0, 0.1);
    HAL_Delay(170);
    reverse(0);
    HAL_Delay(30);
  }
  // reverse(1);
  // reference = calibration();
  reference = calibrationConstant[id];
  if(id %2 ==0){
    mask = 0B00000000;
  } else {
    mask = 0B10000000;
  }

  HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq));  // 2097152U
  changeFreq(1047);

  pwmOutput(0, 0.1);  //??????????????????
  HAL_Delay(500);

  reverse(0);
  changeFreq(20000);
}
