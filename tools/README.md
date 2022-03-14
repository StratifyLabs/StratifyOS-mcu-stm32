To create `core_set_alternate_func.c` for a new MCU, run the following command from this directory. Remember to replace the xml file in the example with the one that matches the new MCU (STM32F413 is shown).

```
python sos_stm32_gpio_parse.py /Applications/STMicroelectronics/STM32CubeMX.app/Contents/Resources/db/mcu/IP/GPIO-STM32F413_gpio_v1_0_Modes.xml
```


`sla` is the latest way to parse and generate these files.

```
sla mcu.parse:interrupts,architecture=stm32,source=/Users/tgil/gitv4/SparkFunThingPlus/SDK/dependencies/StratifyOS-mcu-stm32/include/cmsis/stm32f405xx.h,dest=core_startup.c
```