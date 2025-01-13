package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ButtonMgr {

    public OpMode opMode;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    ControlData[] controlData;
    int tapTime = 500;             // less than = tap, greater than = hold

    public ButtonMgr(OpMode opMode){
        construct(opMode);
    }

    void construct(OpMode opMode){
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        //allocate for # objects based on GPbuttons enum
        controlData = new ControlData[Buttons.values().length * 2];
        //create objects and assign index numbers
        for (int i = 0; i < Buttons.values().length * 2; i++) {
            controlData[i] = new ControlData();
            controlData[i].initData(i);
        }
    }

    public void initLoop() {
        runLoop();
    }

    public void runLoop() {
        updateAll();
    }

    public void updateAll()
    {
        for (ControlData i : controlData) {
            i.update();
        }
    }

    public boolean wasPressed(cButton ctrlButton) {
        return wasPressed(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasPressed(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasPressed;
    }

    public boolean wasReleased(cButton ctrlButton) {
        return wasReleased(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasReleased(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasReleased;
    }

    public boolean wasTapped(cButton ctrlButton) {
        return wasTapped(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasTapped(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasTapped;
    }

    public boolean isHeld(cButton ctrlButton) {
        return isHeld(ctrlButton.controller, ctrlButton.button);
    }
    public boolean isHeld(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].isHeld;
    }

    public boolean isPressed(cButton ctrlButton) {
        return isPressed(ctrlButton.controller, ctrlButton.button);
    }
    public boolean isPressed(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].lastStatus;
    }

    public boolean wasSingleTapped(cButton ctrlButton) {
        return wasSingleTapped(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasSingleTapped(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasSingleTapped;
    }

    public boolean wasDoubleTapped(cButton ctrlButton) {
        return wasDoubleTapped(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasDoubleTapped(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasDoubleTapped;
    }

    public boolean wasHeld(cButton ctrlButton) {
        return wasHeld(ctrlButton.controller, ctrlButton.button);
    }
    public boolean wasHeld(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].wasHeld;
    }

    public boolean isSingleTapHeld(cButton ctrlButton) {
        return isSingleTapHeld(ctrlButton.controller, ctrlButton.button);
    }
    public boolean isSingleTapHeld(int controller, Buttons button) {
        return controlData[getIndex(controller, button)].isSingleTapHeld;
    }

    int getIndex(int controller, Buttons button){
        //This converts an index of 0-31 based on the controller 1-2 and button 0-15
        if (controller < 1 || controller > 2) controller = 0; else controller--;
        return controller * Buttons.values().length + button.ordinal();
    }

    ControlData getAllData(int controller, Buttons button) {
        return controlData[getIndex(controller, button)];
    }

    public boolean getState(int controller, Buttons button, State state) {
        switch (state) {
            //must match the elements in the Actions enum
            case isPressed:         return controlData[getIndex(controller, button)].lastStatus;
            case wasPressed:        return controlData[getIndex(controller, button)].wasPressed;
            case wasReleased:       return controlData[getIndex(controller, button)].wasReleased;
            case wasTapped:         return controlData[getIndex(controller, button)].wasTapped;
            case isHeld:            return controlData[getIndex(controller, button)].isHeld;
            case wasHeld:           return controlData[getIndex(controller, button)].wasHeld;
            case wasSingleTapped:   return controlData[getIndex(controller, button)].wasSingleTapped;
            case wasDoubleTapped:   return controlData[getIndex(controller, button)].wasDoubleTapped;
            case isSingleTapHeld:   return controlData[getIndex(controller, button)].isSingleTapHeld;
            default:                return false;
        }
    }

    class ControlData {
        int index;
        Buttons name;
        boolean lastStatus;
        long lastTime;
        boolean wasPressed;          // Rise
        boolean wasReleased;         // Fall
        boolean isHeld;              // Rise + Long Hold (> tapTime)
        boolean wasHeld;             // Rise + Long Hold (> tapTime) + Fall
        boolean wasTapped;           // Rise + Short Hold (< tapTime) + Fall
        boolean wasSingleTapped;     // Rise + Short Hold (< tapTime) + Fall + Gap (> tapTime)
        boolean wasDoubleTapped;     // Rise + Short Hold (< tapTime) + Fall + Short Gap (< tapTime) x2
        boolean isSingleTapHeld;     // Rise + Short Hold (< tapTime) + Fall + Short Gap (< tapTime) + Rise + Long Hold (> tapTime)
        char tapEventCounter;

        public void initData(int index)
        {
            this.index = index;
            name = Buttons.values()[index % Buttons.values().length];
            lastStatus = false;
            lastTime = System.currentTimeMillis();
            wasTapped = false;
            isHeld = false;
            wasHeld = false;
            wasPressed = false;
            wasReleased = false;
            wasSingleTapped = false;
            wasDoubleTapped = false;
            isSingleTapHeld = false;
            tapEventCounter = 0;
        }

        boolean getReading(int index)
        {
            Gamepad gpad;
            if (index >= Buttons.values().length) {
                index -= Buttons.values().length;
                gpad = gamepad2;
            } else {
                gpad = gamepad1;
            }
            switch (Buttons.values()[index]) {
                //MUST match the elements in the Buttons enum
                case dpad_up:             return gpad.dpad_up;
                case dpad_down:           return gpad.dpad_down;
                case dpad_left:           return gpad.dpad_left;
                case dpad_right:          return gpad.dpad_right;
                case a:                   return gpad.a;
                case b:                   return gpad.b;
                case x:                   return gpad.x;
                case y:                   return gpad.y;
                case start:               return gpad.start;
                case back:                return gpad.back;
                case left_bumper:         return gpad.left_bumper;
                case right_bumper:        return gpad.right_bumper;
                case left_stick_button:   return gpad.left_stick_button;
                case right_stick_button:  return gpad.right_stick_button;
                case left_trigger:        return gpad.left_trigger==1;       // this will treat the triggers like digital buttons
                case right_trigger:       return gpad.right_trigger==1;
                default:                  return false;  //something bad happened
            }
        }

        public void update()
        {
            long currentTime = System.currentTimeMillis();
            long deltaTime = currentTime - lastTime;
            boolean currentState = getReading(index);

            // clear all transient events
            wasPressed = false;
            wasReleased = false;
            wasHeld = false;
            wasTapped = false;
            wasSingleTapped = false;
            wasDoubleTapped = false;
            isHeld = false;

            if (!lastStatus && currentState) {   // change from not pressed to pressed
                wasPressed = true;               // this will last for one loop!
                if (deltaTime < tapTime) {       // indicates released for < tapTime
                    if (tapEventCounter != 0) tapEventCounter++;
                } else {
                    tapEventCounter = 0;
                }
                lastTime = currentTime;          // reset the time
            }
            if (lastStatus && !currentState) {   // change from pressed to not pressed
                wasReleased = true;              // this will last for one loop!
                isSingleTapHeld = false;
                if (deltaTime < tapTime) {
                    wasTapped = true;
                    tapEventCounter++;
                    if (tapEventCounter == 3) {
                        wasDoubleTapped = true;
                        tapEventCounter = 0;
                    }
                } else {
                    wasHeld = true;
                    tapEventCounter = 0;
                }
                lastTime = currentTime;          // reset the time
            }
            if (lastStatus && currentState) {    // still held
                if (deltaTime >= tapTime) {
                    isHeld = true;
                    if (tapEventCounter == 2) {
                        isSingleTapHeld = true;  // will reset when released
                    }
                    tapEventCounter = 0;
                }
            }
            if (!lastStatus && !currentState) {  // still not held
                if (deltaTime >= tapTime) {
                    if (tapEventCounter == 1) {
                        wasSingleTapped = true;
                    }
                    tapEventCounter = 0;
                }
            }
            lastStatus = currentState;
        }
    }

    public static class cButton {
        int controller;
        Buttons button;
        public cButton(int controller, Buttons button){
            this.controller = controller;
            this.button = button;
        }
    }

    public enum Buttons {  //must match what is in ControlData.getReading's switch block
        dpad_up,
        dpad_down,
        dpad_left,
        dpad_right,
        a,
        b,
        x,
        y,
        start,
        back,
        left_bumper,
        right_bumper,
        left_stick_button,
        right_stick_button,
        left_trigger,
        right_trigger
    }

    public enum State {
        isPressed,
        wasPressed,
        wasReleased,
        wasTapped,
        isHeld,
        wasHeld,
        wasSingleTapped,
        wasDoubleTapped,
        isSingleTapHeld
    }
}