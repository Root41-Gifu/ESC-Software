void ESC_Drive() {
  HAL_SYSTICK_Config(SystemCoreClock / (10000U / uwTickFreq));

  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_9);   //! U
  LL_GPIO_SetOutputPin(GPIOC, GPIO_PIN_14);  //! V
  LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_6);   //! W
  for (int i = 0; i < 128; i++) {
		pwmOutput(i);
    HAL_Delay(1);
  }
}

void ESC_initialize() {
  changeFreq(1047);
  unsigned long timer = HAL_GetTick();
  reverse(1);

  while (HAL_GetTick() - timer < 200) {
    LL_TIM_OC_SetCompareCH3(TIM2, waveTable[0] * 0.1);
    LL_TIM_OC_SetCompareCH4(TIM2, waveTable[(10) % 128] * 0.1);
    LL_TIM_OC_SetCompareCH1(TIM2, waveTable[(20) % 128] * 0.1);
  }

  // TODO:キャリブレーション実装しろ
  reverse(0);
  HAL_Delay(1000);
  reverse(1);

  timer = HAL_GetTick();
  while (HAL_GetTick() - timer < 1000) {
    LL_TIM_OC_SetCompareCH3(TIM2, waveTable[0] * 0.1);
    LL_TIM_OC_SetCompareCH4(TIM2, waveTable[(10) % 128] * 0.1);
    LL_TIM_OC_SetCompareCH1(TIM2, waveTable[(20) % 128] * 0.1);
  }

  changeFreq(125000);
}
