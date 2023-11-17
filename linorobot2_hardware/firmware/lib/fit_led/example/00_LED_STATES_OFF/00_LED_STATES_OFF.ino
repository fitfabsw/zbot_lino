#include <FIT_LED.h>

#define LED_PIN_L    6
#define LED_PIN_R    5

#define  NUMBER_OF_LED_L 100
#define  NUMBER_OF_LED_R 100

FITLED led; 

void setup() {
  // put your setup code here, to run once:
  led.setup(NUMBER_OF_LED_L,NUMBER_OF_LED_R,LED_PIN_L,LED_PIN_R,NEO_GRB + NEO_KHZ800);
  led.setLedStatus(LED_STATES_OFF);
}

void loop() {
  // put your main code here, to run repeatedly:
  led.process_led();
}
