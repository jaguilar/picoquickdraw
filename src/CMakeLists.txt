add_executable(quickdraw quickdraw.cc)
target_link_libraries(
  quickdraw 
  PRIVATE pico_stdlib stdc++ hardware_adc hardware_gpio hardware_irq common_nonet)