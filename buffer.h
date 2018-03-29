#pragma once

#include "gcode.h"

int get_buffer_front();

bool get_buffer_empty();

int get_next();

bool get_buffer_full();

gc_block buffer_advance();
gc_block & buffer_current();

gc_block & buffer_last();

bool buffer_add(gc_block gc_add);
