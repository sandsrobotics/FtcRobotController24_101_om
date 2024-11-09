package org.firstinspires.ftc.teamcode.parts.positiontracker.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class OdometryHardware {
    public final DcMotor leftYWheel;
    public final DcMotor rightYWheel;
    public final DcMotor XWheel;

    public final Servo leftYServo;
    public final Servo rightYServo;
    public final Servo XWheelServo;


    public OdometryHardware(DcMotor leftYWheel, DcMotor rightYWheel, DcMotor XWheel, Servo leftY, Servo rightY, Servo x) {
        this.leftYWheel = leftYWheel;
        this.rightYWheel = rightYWheel;
        this.XWheel = XWheel;
        leftYServo = leftY;
        rightYServo = rightY;
        XWheelServo = x;
    }

    public static OdometryHardware makeForOdoBot(HardwareMap map){
        return new OdometryHardware(
                (DcMotor) map.get(MotorSettings.Number.ONE_B.getValue()),
                (DcMotor) map.get(MotorSettings.Number.TWO_B.getValue()),
                //new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD).makeMotor(map),
                //new MotorSettings(MotorSettings.Number.TWO_B, DcMotorSimple.Direction.FORWARD).makeMotor(map),
                new MotorSettings(MotorSettings.Number.THREE_B, DcMotorSimple.Direction.FORWARD).makeMotor(map),
                new ServoSettings(ServoSettings.Number.THREE_B).makeServo(map),
                new ServoSettings(ServoSettings.Number.ONE_B).makeServo(map),
                new ServoSettings(ServoSettings.Number.THREE).makeServo(map)
        );
    }
}
 