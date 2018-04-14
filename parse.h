/* min-vplot: Minimal motion controller for v-plotter. */

#pragma once

static char line[128];

class machine_state;

void parse_line(machine_state & current_state);
