#pragma once

// Compile-time UART conflict detection for ESP32.
//
// RATGDO_ESP32_UART_PORT is set to the value of esp32_uart_num from your YAML.
// RATGDO_NUM_ESPHOME_UARTS is set to the count of 'uart:' components in your YAML.
//
// Both defines are emitted by components/ratgdo/__init__.py during the ESPHome
// code-generation phase.  If this header produces a build error, set
// esp32_uart_num in your ratgdo: configuration to a port not already claimed by
// an ESPHome uart: component.

#ifdef USE_ESP32
#  if defined(RATGDO_NUM_ESPHOME_UARTS) && RATGDO_NUM_ESPHOME_UARTS > 0

#    ifdef USE_ESP_IDF
// ESP-IDF framework: ESPHome's uart: components claim UART_NUM_0, UART_NUM_1, …
// in the order they appear in the YAML.
#      if RATGDO_ESP32_UART_PORT < RATGDO_NUM_ESPHOME_UARTS
#        error "ratgdo: esp32_uart_num conflicts with an ESPHome uart: component. " \
               "In ESP-IDF mode uart: components are allocated from UART_NUM_0 upward. " \
               "Increase esp32_uart_num in your ratgdo: configuration to avoid the conflict."
#      endif

#    else
// Arduino framework: UART_NUM_0 (Serial) is reserved for the logger.
// ESPHome's uart: components claim Serial1 (UART_NUM_1), Serial2 (UART_NUM_2), …
#      if RATGDO_ESP32_UART_PORT >= 1 && RATGDO_ESP32_UART_PORT <= RATGDO_NUM_ESPHOME_UARTS
#        error "ratgdo: esp32_uart_num conflicts with an ESPHome uart: component. " \
               "In Arduino mode uart: components are allocated from UART_NUM_1 upward. " \
               "Adjust esp32_uart_num in your ratgdo: configuration to avoid the conflict."
#      endif

#    endif // USE_ESP_IDF
#  endif // RATGDO_NUM_ESPHOME_UARTS > 0
#endif // USE_ESP32
