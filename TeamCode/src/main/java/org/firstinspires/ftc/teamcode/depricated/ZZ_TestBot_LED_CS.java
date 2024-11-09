package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.QwiicLEDStick;
@Disabled
@TeleOp (name="ZZ_TestBot_LED_CS", group="Test")
public class ZZ_TestBot_LED_CS extends LinearOpMode {

    int ledmode = 1;

    NormalizedColorSensor colorSensor;
    QwiicLEDStick qled = null;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "botSensor");
        qled = hardwareMap.get(QwiicLEDStick.class, "statLed");

        qled.changeLength(20);
        qled.setBrightness(31);
        qled.setAnimationOnOff(1);

        while (!isStarted()) {
            telemetry.addData(">", "Waiting to start...");
            telemetry.update();
            sleep(100);
        }

        qled.turnAllOff();

        float[] hsvValues = new float[3];
        final double gain = 1500; //255;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) ledmode=1;
            if (gamepad1.b) ledmode=2;
            if (gamepad1.y) ledmode=3;

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            double distance = 0;
            if (colorSensor instanceof DistanceSensor) {
                distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red * gain)
                    .addData("Green", "%.3f", colors.green * gain)
                    .addData("Blue", "%.3f", colors.blue * gain);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData("Distance (cm)", "%.3f", distance);

            telemetry.addData("LedMode",ledmode == 1 ? "Direct RGB" : ledmode == 2 ? "Hue to RGB" : ledmode == 3 ? "Bucketed" : "???");

            telemetry.update();

            if (distance > 2) { //no pixel
                qled.setColorGroup(0,5,Color.rgb(0,0,0));
            }
            else {
                switch (ledmode) {
                    case 1: { // map rgb directly to qled
                        int red = (int) (colors.red * gain);
                        int green = (int) (colors.green * gain);
                        int blue = (int) (colors.blue * gain);
                        int max = Math.max(Math.max(red, green), blue);
                        if (max > 255) {
                            red = (int) (255.0 / max * red);
                            green = (int) (255.0 / max * green);
                            blue = (int) (255.0 / max * blue);
                        }
                        qled.setColorGroup(0, 5, Color.rgb(red, green, blue));
                        break;
                    }
                    case 2: { // map hue into an rgb value
                        float hue=hsvValues[0]/120;  //red=0,green=1,blue=3
                        int red = 0;
                        int green = 0;
                        int blue = 0;
                        if (hue<1) {  //red to green
                            red = (int)(255*(1-hue));
                            green = (int)(255*hue);
                            blue = 0;
                        }
                        if (hue>=1 && hue < 2) {  //green to blue
                            red = 0;
                            green = (int)(255*(2-hue));
                            blue = (int)(255*(hue-1));
                        }
                        if (hue>=2) {  //blue back arounf to red
                            red = (int)(255*(hue-2));
                            green = 0;
                            blue = (int)(255*(3-hue));
                        }
                        qled.setColorGroup(0, 5, Color.rgb(red, green, blue));
                        break;
                    }
                    case 3: { // bucket pixels based on hue
                        int hue = (int) hsvValues[0];
                        int pixel = 0;
                        if (hue < 100) pixel = 1; //yellow
                        if (hue >= 100 && hue <= 140) pixel = 2; //green
                        if (hue > 180) pixel = 3; //purple
                        switch (pixel) {
                            case 1:  //yellow
                                qled.setColorGroup(0, 5, Color.rgb(127, 50, 0));  //yellow
                                break;
                            case 2:  //green
                                qled.setColorGroup(0, 5, Color.rgb(0, 127, 0));  //green
                                break;
                            case 3:  //purple
                                qled.setColorGroup(0, 5, Color.rgb(120, 0, 90));  //purple
                                break;
                            default:  //white
                                qled.setColorGroup(0, 5, Color.rgb(48, 45, 36));  //white
                                break;
                        }
                        break;
                    }
                    default:
                        break;
                }
            }

            sleep(200);
        }
    }

}