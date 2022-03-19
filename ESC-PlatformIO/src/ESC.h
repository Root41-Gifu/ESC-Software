#define timeLoop(x)      \
  timer = HAL_GetTick(); \
  while (HAL_GetTick() - timer < x)

unsigned long timer;

void ESC_Drive() {
  electricalAngle = getElectricalAngle();

  if (electricalAngle <= _electricalAngle) {
    if (electricalAngle <= _electricalAngle - 10) {
    } else {
      electricalAngle = _electricalAngle;
    }
  }

  pwmOutput((electricalAngle + 32) % 128, 0.2);

  _electricalAngle = electricalAngle;

  HAL_Delay(100);
}

int calibration(void) {
  changeFreq(125000);

  reverse(1);

  HAL_SYSTICK_Config(SystemCoreClock / (10000U / uwTickFreq));  // 2097152U

  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_9);   //! U
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_14);  //! V
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_6);   //! W

  int mechanicalAngle;

  while (true) {
    for (int i = 0; i < 128; i += 1) {
      mechanicalAngle = encoderRead();
      if (1500 <= mechanicalAngle && mechanicalAngle <= 1520) {
        goto END;
      }

      pwmOutput(i, 0.2);
      HAL_Delay(1);
    }
  }
END:
  timeLoop(500) { pwmOutput(0, 0.3); }
  timeLoop(200) { pwmOutput(32, 0.1); }
  timeLoop(500) { pwmOutput(0, 0.3); }
  timeLoop(200) { pwmOutput(128 - 32, 0.1); }
  timeLoop(3000) { pwmOutput(0, 0.4); }
  reverse(1);
  mechanicalAngle = 0;
  for (int i = 0; i < 20; i++) {
    pwmOutput(0, 0.4);
    mechanicalAngle = (mechanicalAngle * i + encoderRead()) / (i + 1);
    HAL_Delay(500);
  }
  return mechanicalAngle;
}

void ESC_initialize() {
  int doremi[3] = {1047, 1175, 1319};
  for (int i = 0; i < 3; i++) {
    changeFreq(doremi[i]);
    timeLoop(100) {
      reverse(1);
      pwmOutput(0, 0.1);
    }
    // reverse(0);
  }
  reverse(0);
  HAL_Delay(100);
  reference = calibration();

  HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq));  // 2097152U
  changeFreq(1047);
  timeLoop(800) {
    pwmOutput(0, 0.1);  //ブザーならす
  }
  reverse(0);
  // changeFreq(30000);
  changeFreq(30000);
}
