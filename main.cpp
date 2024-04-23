#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <hardware/pio.h>
#include "hardware/clocks.h"
#include "debounce.pio.h"
#include "pico/multicore.h"

/******************BEGIN CONSTANTS declarations**************************/

const uint BUTTON_LEFT = 29;               //input LEFT
const uint BUTTON_LEFT_SM_OUTPUT =25;      //used as outputs from PIO sm - debounced button presses
const uint BUTTON_LEFT_SOCD_PIN = 13;      //output for SOCD cleaned input LEFT


const uint BUTTON_RIGHT = 28;              //input RIGHT
const uint BUTTON_RIGHT_SM_OUTPUT = 24;    //used as outputs from PIO sm - debounced button presses
const uint BUTTON_RIGHT_SOCD_PIN = 12;     //output for SOCD cleaned input RIGHT


const uint BUTTON_UP = 0;                  //input UP
const uint BUTTON_UP_SM_OUTPUT = 23;       //used as outputs from PIO sm - debounced button presses
const uint BUTTON_UP_SOCD_PIN = 10;        //output for SOCD cleaned input UP


const uint BUTTON_DOWN = 1;                //input DOWN
const uint BUTTON_DOWN_SM_OUTPUT = 22;     //used as outputs from PIO sm - debounced button presses
const uint BUTTON_DOWN_SOCD_PIN = 9;       //output for SOCD cleaned input DOWN


const PIO debounce_pio = pio0;              //PIO instance
const float pio_freq = 2000;                //PIO clock freq
uint sm0;                                   //LEFT debounce state machine
uint sm1;                                   //RIGHT debounce state machine
uint sm2;                                   //UP debounce state machine
uint sm3;                                   //DOWN debounce state machine
uint offset;                                //PIO program memory offset
float clock_div = 
    (float)clock_get_hz(clk_sys) / pio_freq;//PIO instance clock divider

/*******************END CONSTANTS declarations***************************/


//setup function for PIO debouncing program
void debounce_pio_setup(bool clock_div_enable = false, float clock_div = 1)
{
    //pull all BUTTON inputs for state machines up
    gpio_pull_up(BUTTON_LEFT);
    gpio_pull_up(BUTTON_RIGHT);
    gpio_pull_up(BUTTON_UP);
    gpio_pull_up(BUTTON_DOWN);

    //claim state machines
    sm0 = pio_claim_unused_sm(debounce_pio, true);
    sm1 = pio_claim_unused_sm(debounce_pio, true);
    sm2 = pio_claim_unused_sm(debounce_pio, true);
    sm3 = pio_claim_unused_sm(debounce_pio, true);

    //load programs into pio memory
    offset = pio_add_program(debounce_pio, &debounce_program);

    //initialize state machines (this function written in debounce.pio, passed thru to debounce.pio.h)
     debounce_program_init(debounce_pio, sm0, offset, BUTTON_LEFT_SM_OUTPUT, BUTTON_LEFT, clock_div);
     debounce_program_init(debounce_pio, sm1, offset, BUTTON_RIGHT_SM_OUTPUT, BUTTON_RIGHT, clock_div);
     debounce_program_init(debounce_pio, sm2, offset, BUTTON_UP_SM_OUTPUT, BUTTON_UP, clock_div);
     debounce_program_init(debounce_pio, sm3, offset, BUTTON_DOWN_SM_OUTPUT, BUTTON_DOWN, clock_div);

    //start state machines
     pio_sm_set_enabled(debounce_pio, sm0, true);
     pio_sm_set_enabled(debounce_pio, sm1, true);
     pio_sm_set_enabled(debounce_pio, sm2, true);
     pio_sm_set_enabled(debounce_pio, sm3, true);

    return;
};


//setup function for GPIO pins, namely SOCD outputs for main program
void gpio_setup()
{
    //initialize gpio pins
    gpio_init_mask(
        (1<<BUTTON_LEFT_SOCD_PIN)|
        (1<<BUTTON_RIGHT_SOCD_PIN)|
        (1<<BUTTON_UP_SOCD_PIN)|
        (1<<BUTTON_DOWN_SOCD_PIN)
    );  

    //set pins to outputs mask – Bitmask of GPIO to set to input, as bits 0-29
    /*mask – Bitmask of GPIO to set to input, as bits 0-29
        value – Values to set For each 1 bit in "mask", switch that pin to the direction
        given by corresponding bit in "value", leaving other pins unchanged. 
        E.g. gpio_set_dir_masked(0x3, 0x2); -> set pin 0 to input, pin 1 to output, simultaneously.*/
    gpio_set_dir_masked(
        (
            (1<<BUTTON_LEFT_SOCD_PIN)|
            (1<<BUTTON_RIGHT_SOCD_PIN)|
            (1<<BUTTON_UP_SOCD_PIN)|
            (1<<BUTTON_DOWN_SOCD_PIN)
        ),
        (
            (1<<BUTTON_LEFT_SOCD_PIN)|
            (1<<BUTTON_RIGHT_SOCD_PIN)|
            (1<<BUTTON_UP_SOCD_PIN)|
            (1<<BUTTON_DOWN_SOCD_PIN)
        )
    );

    stdio_init_all();

    return;
};


//SOCD cleaning function - takes two opposite cardinals, a & b, and outputs A == A & !B, B == B & !A (A+B = Neutral)
void socd_clean(uint gpio_a_debounced_input, uint gpio_b_debounced_input, uint socd_output_a, uint socd_output_b){

    uint32_t mask = gpio_get_all(); //sample the pins, store in vars
    bool gpio_a_debounced_val = (mask>>gpio_a_debounced_input)%2; 
    bool gpio_b_debounced_val = (mask>>gpio_b_debounced_input)%2;

    //if both buttons pressed, drom both low
    if(gpio_a_debounced_val && gpio_b_debounced_val) 
    {
        gpio_put_masked( (1 << socd_output_a) | (1 << socd_output_b), false );
    }

    //otherwise, set corresponding pin high
    else{
        gpio_put_masked(
            ( (1 << socd_output_a) | (1 << socd_output_b) ),
            ( (gpio_a_debounced_val << socd_output_a) | (gpio_b_debounced_val << socd_output_b) )
        );
    };
    return; 
 };


//UP and DOWN inputs cleaned & monitored on core1, so each axis can be checked & cleaned independently.
void core1_main(){
    while (true){
        socd_clean(BUTTON_UP_SM_OUTPUT, BUTTON_DOWN_SM_OUTPUT, BUTTON_UP_SOCD_PIN, BUTTON_DOWN_SOCD_PIN);
    }
};


int main(){
    gpio_setup();
    debounce_pio_setup();
    multicore_launch_core1(&core1_main);
    while (true){
        socd_clean(BUTTON_LEFT_SM_OUTPUT, BUTTON_RIGHT_SM_OUTPUT, BUTTON_LEFT_SOCD_PIN, BUTTON_RIGHT_SOCD_PIN);
    };
    return 0;
};

