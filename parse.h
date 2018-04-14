/* min-vplot: Minimal motion controller for v-plotter. */

#pragma once

class machine_state;

void parse_line(char * line, machine_state & current_state);
