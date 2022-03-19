// NOTE:Arduinoからの引用
#define constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

extern long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//周波数変更
void changeFreq(unsigned long freq) {
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  TIM_InitStruct.Prescaler = (125000 / freq) - 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 255;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIM2, LL_TIM_CHANNEL_CH4);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_TIM_EnableCounter(TIM2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
}

char waveTable[] = {
    127, 133, 139, 146, 152, 158, 164, 170, 176, 181, 187, 192, 198, 203, 208,
    212, 217, 221, 225, 229, 233, 236, 239, 242, 244, 247, 249, 250, 252, 253,
    253, 254, 254, 254, 253, 253, 252, 250, 249, 247, 244, 242, 239, 236, 233,
    229, 225, 221, 217, 212, 208, 203, 198, 192, 187, 181, 176, 170, 164, 158,
    152, 146, 139, 133, 127, 121, 115, 108, 102, 96,  90,  84,  78,  73,  67,
    62,  56,  51,  46,  42,  37,  33,  29,  25,  21,  18,  15,  12,  10,  7,
    5,   4,   2,   1,   1,   0,   0,   0,   1,   1,   2,   4,   5,   7,   10,
    12,  15,  18,  21,  25,  29,  33,  37,  42,  46,  51,  56,  62,  67,  73,
    78,  84,  90,  96,  102, 108, 115, 121};

void reverse(int a) {  //反転相を制御するよ
  if (a == 1) {
    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_9);   //! U
    LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_14);  //! V
    LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_6);   //! W
  } else {
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_9);   //! U
    LL_GPIO_ResetOutputPin(GPIOC, GPIO_PIN_14);  //! V
    LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_6);   //! W
  }
}

void pwmOutput(int a, float power) {  //各相に電圧を印加するよ
  LL_TIM_OC_SetCompareCH3(TIM2, waveTable[a] * power);
  LL_TIM_OC_SetCompareCH4(TIM2, waveTable[(a + 43) % 128] * power);
  LL_TIM_OC_SetCompareCH1(TIM2, waveTable[(a + 85) % 128] * power);
}

int encoderRead(void) {
  HAL_ADC_Start(&hadc);

  // HAL_ADC_Stop(&hadc);
  int s = HAL_ADC_PollForConversion(&hadc, 3);
  int value = 0;
  if (s == HAL_OK) {
    value = HAL_ADC_GetValue(&hadc);
  }

  return 4095 - value;
}

void ESC_activate(void) {
  bool startFlag = false;
  while (!startFlag) {
    HAL_UART_Receive_IT(&huart2, serialBuffer, 1);
    // HAL_UART_Transmit_IT(&huart2, buffer, 1);
    gUartReceived = 0;

    if (serialBuffer[0] != 0) {
      startFlag = true;
    }
  }
}

int convertToElectricalAngle(
    const int _mechanicalAngle) {  //電気角に変換 1byteで返す
  int _electricalAngle = constrain(_mechanicalAngle, 0, 4095) % 585;
  _electricalAngle = constrain(map(_electricalAngle, 0, 585, 0, 127), 0, 127);

  return _electricalAngle;
}

char getElectricalAngle(void) {
  int angle = encoderRead() + 4096 - reference;  // 1447
  angle %= 4096;
  angle = convertToElectricalAngle(angle);

  return (char)angle;
}