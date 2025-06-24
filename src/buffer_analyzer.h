#ifndef BUFFER_ANALYZER_H
#define BUFFER_ANALYZER_H

#include <Arduino.h>
#include "core1.h" // Per definizioni come Bit e costanti

void analyze_buffer_32(const uint16_t* buffer, size_t length);

#endif