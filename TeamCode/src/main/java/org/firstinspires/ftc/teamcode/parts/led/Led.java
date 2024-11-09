package org.firstinspires.ftc.teamcode.parts.led;

import android.graphics.Color;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.QwiicLEDStick;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class Led extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {

    public QwiicLEDStick leds;

    boolean bottomOn = false;
    boolean topOn = false;
    int botColor = 0;
    int topColor = 0;

    public Led(Robot robot){
        super(robot, "Leds");
    }

    @Override
    public void onRun() {

    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        leds = parent.opMode.hardwareMap.get(QwiicLEDStick.class, "statLed");
    }

    @Override
    public void onStart() {
        leds.setBrightness(10);//max of 31
        leds.turnAllOff();
    }

    @Override
    public void onStop() {
        leds.setColor(Color.BLACK);
        leds.turnAllOff();
    }

    public void setBottomGroup(boolean turnOn){
        if (turnOn != bottomOn) {
            bottomOn = turnOn;
            if (turnOn) leds.setColorGroup(0, 5, Color.BLUE);
            else leds.setColorGroup(0, 5, Color.BLACK);
        }
    }

    public void setTopGroup(boolean turnOn){
        if (turnOn != topOn) {
            topOn = turnOn;
            if (turnOn) leds.setColorGroup(5, 5, Color.GREEN);
            else leds.setColorGroup(5, 5, Color.BLACK);
        }
    }

    public void setBottomGroup2(int color){
        if (color != botColor) {
            botColor = color;
            switch (color) {
                case 1:  //yellow
                    leds.setColorGroup(0, 5, Color.rgb(127, 50, 0));  //yellow
                    break;
                case 2:  //green
                    leds.setColorGroup(0, 5, Color.rgb(0, 127, 0));  //green
                    break;
                case 3:  //purple
                    leds.setColorGroup(0, 5, Color.rgb(120, 0, 90));  //purple
                    break;
                case 4:  //white
                    leds.setColorGroup(0, 5, Color.rgb(48, 45, 36));  //white
                    break;
                default:  //off
                    leds.setColorGroup(0, 5, Color.BLACK);  //white
                    break;
            }
        }
    }

    public void setTopGroup2(int color){
        if (color != topColor) {
            topColor = color;
            switch (color) {
                case 1:  //yellow
                    leds.setColorGroup(5, 10, Color.rgb(127, 50, 0));  //yellow
                    break;
                case 2:  //green
                    leds.setColorGroup(5, 10, Color.rgb(0, 127, 0));  //green
                    break;
                case 3:  //purple
                    leds.setColorGroup(5, 10, Color.rgb(120, 0, 90));  //purple
                    break;
                case 4:  //white
                    leds.setColorGroup(5, 10, Color.rgb(48, 45, 36));  //white
                    break;
                default:  //off
                    leds.setColorGroup(5, 10, Color.BLACK);  //white
                    break;
            }
        }
    }
}
