#define timeLoop(x)      \
  timer = HAL_GetTick(); \
  while (HAL_GetTick() - timer < x)

unsigned long timer;

void ESC_Drive() {
  HAL_SYSTICK_Config(SystemCoreClock / (20000U / uwTickFreq));

  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_9);   //! U
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_14);  //! V
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_6);   //! W
  for (int i = 0; i < 128; i++) {
    pwmOutput(i, 0.3);
    HAL_Delay(1);
  }
}

void ESC_initialize() {
  changeFreq(1047);

  timeLoop(100) {
    reverse(1);
    pwmOutput(0, 0.1);
  }

  // TODO:キャリブレーション実装しろ
  reverse(0);
  HAL_Delay(1000);
  reverse(1);

  timeLoop(1000) {
    pwmOutput(0, 0.2);  //ブザーならす
  }

  changeFreq(125000);
}
