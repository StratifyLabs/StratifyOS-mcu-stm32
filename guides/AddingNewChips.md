

- Update `CMakeLists.txt`
- Add arch definition to `StratifyOS/include/mcu/arch.h`
- Add `include/cmsis/<header>.h`
  - Update path to `core_cm4.h` in `include/cmsis/<header>.h`
- Add `<chip>` folder to `src`
- Parse GPIO xml to create `core_set_alternate_func.c`
- Parse `include/cmsis/<header>.h` to create list of ISRs and copy to core_startup.c
- Update `include/stm32_arch.h` with correct definitions
- Create `include/mcu_<chip>.h` header file
- Add linker files
