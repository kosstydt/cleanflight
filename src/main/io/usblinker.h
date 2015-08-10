#pragma once


typedef struct {
  GPIO_TypeDef* gpio;
  uint32_t pinpos;
} escHardware_t;

void USBLinker(uint16_t escIndex);
