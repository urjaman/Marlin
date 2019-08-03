
#pragma once

struct LEDColor;
typedef LEDColor LEDColor;

void ledthing_init(void);
void ledthing_set_led_color(const LEDColor &color);
void ledthing_handle_status(void);
