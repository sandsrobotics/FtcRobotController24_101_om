package org.firstinspires.ftc.teamcode.parts.drive.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class DriveHardware {
    ///////////
    //objects//
    ///////////
    public final DcMotorEx topLeftMotor;
    public final DcMotorEx topRightMotor;
    public final DcMotorEx bottomLeftMotor;
    public final DcMotorEx bottomRightMotor;


    public DriveHardware(HardwareMap hardwareMap, MotorSettings topLeftMotorSettings, MotorSettings topRightMotorSettings, MotorSettings bottomLeftMotorSettings, MotorSettings bottomRightMotorSettings) {
        topLeftMotor = topLeftMotorSettings.makeExMotor(hardwareMap);
        topRightMotor = topRightMotorSettings.makeExMotor(hardwareMap);
        bottomLeftMotor = bottomLeftMotorSettings.makeExMotor(hardwareMap);
        bottomRightMotor = bottomRightMotorSettings.makeExMotor(hardwareMap);

        List<DcMotorEx> motors = Arrays.asList(topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor);
        for(DcMotorEx motor : motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

    }

    public DriveHardware(DcMotorEx topLeftMotor, DcMotorEx topRightMotor, DcMotorEx bottomLeftMotor, DcMotorEx bottomRightMotor){
        this.topLeftMotor = topLeftMotor;
        this.topRightMotor = topRightMotor;
        this.bottomLeftMotor = bottomLeftMotor;
        this.bottomRightMotor = bottomRightMotor;
    }

    public static DriveHardware makeDefault(HardwareMap hardwareMap){
        ////////////
        //settings//
        ////////////
        MotorSettings topLeftMotorSettings = new MotorSettings(MotorSettings.Number.ZERO, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings topRightMotorSettings = new MotorSettings(MotorSettings.Number.ONE, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomLeftMotorSettings = new MotorSettings(MotorSettings.Number.TWO, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomRightMotorSettings = new MotorSettings(MotorSettings.Number.THREE, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);

        return new DriveHardware(hardwareMap, topLeftMotorSettings, topRightMotorSettings, bottomLeftMotorSettings, bottomRightMotorSettings);
    }

    public static DriveHardware lkTestChassis(HardwareMap hardwareMap){
        ////////////
        //settings//
        ////////////
        MotorSettings topLeftMotorSettings = new MotorSettings(MotorSettings.Number.ZERO, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings topRightMotorSettings = new MotorSettings(MotorSettings.Number.ONE, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomLeftMotorSettings = new MotorSettings(MotorSettings.Number.TWO, DcMotorSimple.Direction.FORWARD, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomRightMotorSettings = new MotorSettings(MotorSettings.Number.THREE, DcMotorSimple.Direction.REVERSE, DcMotorEx.ZeroPowerBehavior.BRAKE, DcMotorEx.RunMode.RUN_USING_ENCODER, 0);

        return new DriveHardware(hardwareMap, topLeftMotorSettings, topRightMotorSettings, bottomLeftMotorSettings, bottomRightMotorSettings);
    }
}
