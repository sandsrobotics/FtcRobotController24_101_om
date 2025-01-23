package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.core.Robot;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class LedTasks {
    public final Group ledTasksGroup;
    public final TimedTask yellowGood;
    public final TimedTask yellowBad;
    public final TimedTask isYellow;
    public final TimedTask isRed;
    public final TimedTask isBlue;
    public final TimedTask ejecting;
    private final Intake intake;
    private final Robot robot;
    public final ServoSSR rgbIndicator;

    public LedTasks(Intake intake, Robot robot) {
        this.intake = intake;
        this.robot = robot;
        ledTasksGroup = new Group("led", intake.getTaskManager());
        yellowGood = new TimedTask("yellow good", ledTasksGroup);
        yellowBad = new TimedTask("yellow bad", ledTasksGroup);
        isYellow = new TimedTask("is yellow", ledTasksGroup);
        isBlue = new TimedTask("is blue", ledTasksGroup);
        isRed = new TimedTask("is red", ledTasksGroup);
        ejecting = new TimedTask("ejecting", ledTasksGroup);
        rgbIndicator = new ServoSSR(robot.opMode.hardwareMap.get(Servo.class, "servo5B"));
    }

    public void constructAllLedTasks() {

        /* == Task: yellowGood == */
        yellowGood.autoStart = false;
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color));
        yellowGood.addDelay(250);
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        yellowGood.addDelay(100);
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color));
        yellowGood.addDelay(250);
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        yellowGood.addDelay(100);
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Green.color));
        yellowGood.addDelay(500);
        yellowGood.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        /* == Task: yellowBad == */
        yellowBad.autoStart = false;
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color));
        yellowBad.addDelay(250);
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        yellowBad.addDelay(100);
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color));
        yellowBad.addDelay(250);
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        yellowBad.addDelay(100);
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color));
        yellowBad.addDelay(500);
        yellowBad.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        isYellow.autoStart = false;
        isYellow.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Yellow.color));
        isYellow.addDelay(1000);
        isYellow.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        isRed.autoStart = false;
        isRed.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Red.color));
        isRed.addDelay(1000);
        isRed.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        isBlue.autoStart = false;
        isBlue.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Blue.color));
        isBlue.addDelay(1000);
        isBlue.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        ejecting.autoStart = false;
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Green.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
//        ejecting.addDelay(100);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Green.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
//        ejecting.addDelay(100);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Green.color));
//        ejecting.addDelay(250);
//        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
        ejecting.addDelay(350);
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        ejecting.addDelay(150);
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
        ejecting.addDelay(350);
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));
        ejecting.addDelay(150);
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Violet.color));
        ejecting.addDelay(350);
        ejecting.addStep(() -> rgbIndicator.setPosition(rgbIndicatorColor.Off.color));

        /* ========== end of constructAllIntakeTasks() ========== */
    }

    public enum rgbIndicatorColor {
        Off (0.0),
        Red (0.279),
        Orange (0.333),
        Yellow (0.388),
        Sage (0.444),
        Green (0.500),
        Azure (0.555),
        Blue (0.611),
        Indigo (0.666),
        Violet (0.715), //(0.722),
        White (1.0);

        private final double color;

        rgbIndicatorColor(double color) {
            this.color = color;
        }
    }
}
