#include "EMFButton.h"
#include "tools.h"
#include "pico/stdlib.h"
#include "stdio.h"


// state:
// avait = not pressed/released since some time
// pressed = pressed since less than hold delay
// held = just pressed for more than hold delay (just once)
// released =  released when it was in pressed state  

void EMFButton::tick() {
  _clicked = 0;
  _held = 0;
  _hold = 0;
  _released = 0;
  _clicksEnd = 0;
  _clicksWithHeld = 0;
  _clicksWithHold = 0;
  uint32_t t = millisRp();
  //bool reading = digitalRead(_pin) == (_pinmode) ? HIGH : LOW;
  bool reading = get_bootsel_button();
  if (reading != _lastState) {
    _lastChange = t;
  }
  if (t - _lastChange > EMFB_DEB_TIMER) {                   // debounce time expired
    if (reading == (_pinclosed) ? _pressed : !_pressed) {   // if reading == _pressed
      _pressed = (_pinclosed) ? !reading : reading;   // _pressed = not reading because it is normally a contact to Gnd so 0 = pressed
    }

    if (_pressed) {
      switch (mode) {
        case await:
          mode = pressed;
          _timer = t;
          _clicks = 1;
          _clicked = 1;
          break;
        case pressed:
          if (t - _timer >= EMFB_HOLD_TIMER) {
            mode = held;   // just enter held start         
            _timer = t;   // timer = when hold is detected
            _held = 1;    // is held 
            _clicksWithHeld = _clicks - 1; // 
          }
          break;
        case held:
          _hold = 1;        // hold at least one more tick
          _clicksWithHold = _clicks - 1;
          break;
        case released:
          mode = pressed;
          _timer = t;
          _clicks ++;   // count nbr of click
          _clicked = 1; // set flag to say that it has been clicked 
          break;
      }
    } else {
      switch (mode) {
        case pressed:  
            if (_clicks == 15) {
                mode = await;
                _clicksEnd = _clicks;
                _clicks = 0;
            } else { 
                mode = released;
            }    
            _timer = t;
            _released = 1;
            break;
        case held:
            mode = await;
            _timer = t;
            _clicks = 0;
            _released = 1;
            break;
        case released:
            if (t - _timer >= EMFB_RELEASE_TIMER) {
                mode = await;
                _timer = t;
                _clicksEnd = _clicks;
                _clicks = 0;
            }
            break;
        case await: // added to avoid a compiler warning 
            break;     
      }
    }
  }
  _lastState = reading;
}

extern EMFButton btn;

void debugBootButton(){
   // update the state of the boot button and call some event functions
    btn.tick();
    //if (btn.isPressed()) {  // // current state (repeat)
    //  printf("is pressed\n");
    //}
    
    if (btn.isClicked()) {  // just pressed after await or release state (do not repeat)
        printf("click %d\n", btn.getClicks());
    }

    if ( btn.isHeld() ){ // just pressed during more than x (time for hold) (do not repeat)
        printf("is held\n");
    }
    // isHold()          // keep long pressing (repeat)
        
    if ( btn.hasClicks() ){
        printf("has clicks %d\n", btn.hasClicks()); // button is released after having been pressed, return gives number of clicks
    }

    if ( btn.hasClicksWithHeld() ){
        printf("has clicks With Held %d\n", btn.hasClicksWithHeld()); // seems to react after several clicks + a hold
    }

    //if ( btn.hasClicksWithHold() ){
    //  printf("has clicks With Hold\n"); // repeat
    //}

    if ( btn.isReleased() ){
        printf("is released\n");   // just released (only once ; does not remain) 
    }
    
    //if ( btn.hasOnlyHold() ){
    //  printf("has only hold\n"); // seems to react when hold during the first click; then repeat
    //}

    //if ( btn.hasHoldAndSingle() ){
    //  printf("has hold and single\n"); // seems to react when hold after after one click + hold ; then repeat
    //}
    //if ( btn.hasHoldAndDouble() ){
    //  printf("has hold and double\n"); // seems to react when hold after 2 click + hold ; then repeat
    //}
    
    if ( btn.hasOnlyHeld() ){
        printf("has only held\n"); // seems to react after 
    }
    if ( btn.hasHeldAndSingle() ){
        printf("has held and single\n"); // seems to react after 
    }
    if ( btn.hasHeldAndDouble() ){
        printf("has held and double\n"); // seems to react after 
    }
    

    //if ( btn.isHold() ){
    //  printf("hold\n"); // repeat when hold
    //}
}    