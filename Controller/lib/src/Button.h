#ifndef BUTTON_H
#define BUTTON_H
#include <Arduino.h>

int BUTTON_PRESS_DEBOUNCE_DELAY = 50;
extern pointerFunction mainLoop;

class Button
{
    int currentState = 0;
    int prevSteadyState = 0;
    int prevFlickerState;
    unsigned long pressTime;
    bool isPressed = false;
    
    uint8_t pin;
    uint8_t mode;
public:
    Button(uint8_t pin, uint8_t mode = INPUT_PULLUP)
    {
        PinMode(pin, mode);
    }
    
    // (Optional) Not required in setup. Use to change pinmode at runtime 
    void PinMode(uint8_t pin, uint8_t mode = INPUT_PULLUP)
    {
        this->pin = pin;
        this->mode = mode;
        pinMode(pin, mode);
    }

    void Update()
    {
        currentState = digitalRead(pin);

        if (currentState != prevFlickerState) {
            pressTime = millis();
            prevFlickerState = currentState;
        } 
        if ((millis() - pressTime) > BUTTON_PRESS_DEBOUNCE_DELAY) {
            //if steady state changed
            if (currentState != prevSteadyState) {
                isPressed = (currentState == 1);
                prevSteadyState = currentState;
                pressTime = millis();
            }
        }
    }
    
    bool IsPressed()
    {
        return isPressed;        
    }
};

#endif