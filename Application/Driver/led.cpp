#include "led.h"

void Led::init()
{
	GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

 

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin1 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin1; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin2 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin2; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin3 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin3; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin4 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin4; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin5 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin5; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin6 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin6; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin7 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin7; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PtPin8 */
  GPIO_InitStruct.Pin = LED_GREEN_BAR_Pin8; 
  HAL_GPIO_Init(LED_GREEN_BAR_GPIO_Port, &GPIO_InitStruct);
}

void Led::red_on_off(bool status)
{
	if(status == true){ 
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
}

void Led::green_on_off(bool status)
{
	if(status == true){ 
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
}


void Led::green_bar_on_off(unsigned char index)
{
	bool led_bar[8];
	int i = 0;
	unsigned char temp = 0x80;
	for(i = 0; i < 8; i++)
	{
		led_bar[i] = ((index & temp) == temp);
		temp = temp >> 1;
	}
	
	if(led_bar[0] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin1, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin1, GPIO_PIN_RESET);
	}
	
	if(led_bar[1] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin2, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin2, GPIO_PIN_RESET);
	}
	
	if(led_bar[2] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin3, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin3, GPIO_PIN_RESET);
	}
	
	if(led_bar[3] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin4, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin4, GPIO_PIN_RESET);
	}
	
	if(led_bar[4] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin5, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin5, GPIO_PIN_RESET);
	}
	
	if(led_bar[5] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin6, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin6, GPIO_PIN_RESET);
	}
	
	if(led_bar[6] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin7, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin7, GPIO_PIN_RESET);
	}
	
	if(led_bar[7] == true){ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin8, GPIO_PIN_SET);
	}else{ 
		HAL_GPIO_WritePin(LED_GREEN_BAR_GPIO_Port, LED_GREEN_BAR_Pin8, GPIO_PIN_RESET);
	}
}
