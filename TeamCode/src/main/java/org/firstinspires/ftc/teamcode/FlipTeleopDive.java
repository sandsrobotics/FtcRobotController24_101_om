package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveSettings;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake.FlipbotSettings;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleop;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleopDemo;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="14273.3 Arcade RED", group="B14273")
public class FlipTeleopDive extends LinearOpMode {
    double tileSide = 23.5;
    Drive drive;
    Robot robot;
    Intake intake;
    PositionSolver positionSolver;
    PositionTracker pt;
    Pinpoint odo;
    Vector3 fieldStartPos = new Vector3(-14.375,-62,90);  //for teleOp, this shouldn't be relevant
    boolean testModeReverse = false;
//    public long hangTime=0;

    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeArcade1(robot));
    }

    @Override
    public void runOpMode() {
        extraSettings();   // LK 20250602 Moved to top
        FlipbotSettings.setTeleOp();
        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot = new Robot(this);
        new BulkRead(robot);
        drive = new Drive(robot);
//        drive = new Drive(robot, DriveSettings.makeDefault(), DriveHardware.lkTestChassis(robot.opMode.hardwareMap));
        initTeleop();

        final Vector3 p_atObsZone = new Vector3(33.5, -56.5, 90); // p_12: Position near ObsZone for Pickup-Specimen.
        Vector3 tempPosition = FlipbotSettings.getRobotPosition();

        if (tempPosition.X == 0.0 && tempPosition.Y == 0.0 && tempPosition.Z == 0.0)  {
            FlipbotSettings.storeRobotPosition(p_atObsZone);
        }

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
//               100, new Vector3(2,2,2), fieldStartPos);
                100, new Vector3(2,2,2), FlipbotSettings.getRobotPosition());
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
     // XRelativeSolver solver = new XRelativeSolver(drive);

//       EncoderTracker et = new EncoderTracker(pt);
//       pt.positionSourceId = EncoderTracker.class;

        odo = new Pinpoint(pt, false, "pinpoint",
                -56.0, 52.0, 13.26291192f,
                GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pt.positionSourceId = Pinpoint.class;
        positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise
        positionSolver.setSettings(PositionSolverSettings.specimenAssistSettings);

        intake = new Intake(robot);
        if (!FlipbotSettings.isDemoMode) new IntakeTeleop(intake);
        else new IntakeTeleopDemo(intake);

        robot.init();
//        odo.setPosition(fieldStartPos);

//        long timer = System.currentTimeMillis() + 2500;
//        while (odo.getValidPosition() == null && System.currentTimeMillis() <= timer) {
//            telemetry.addLine("Waiting for Pinpoint...");
//            telemetry.update();
//            //todo: What to do if it doesn't initialize?
//        }
        try {
            odo.setPosition(FlipbotSettings.getRobotPosition());
        } catch (Exception e) {
            telemetry.addLine("Exception while odo.setPosition; Ignoring");
            telemetry.update();
        }

        intake.getHardware().robotHangMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.getHardware().robotHangMotor.setPower(-1);

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
            telemetry.addData("Not Started", "Not Started");

            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasDoubleTapped)) {
                drive.lkUpdateConfig(DriveSettings.makeDefault(), DriveHardware.lkTestChassis(robot.opMode.hardwareMap));
                testModeReverse = true;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_down, ButtonMgr.State.wasSingleTapped)) {
                drive.lkUpdateConfig(DriveSettings.makeDefault(), DriveHardware.makeDefault(robot.opMode.hardwareMap));
                testModeReverse = false;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.dpad_up, ButtonMgr.State.wasTapped)) {
                odo.setPosition(fieldStartPos);
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped) ||
                    robot.buttonMgr.getState(2, ButtonMgr.Buttons.x, ButtonMgr.State.wasTapped)) {
                intake.initializeServos();
            }
            telemetry.addData("Drive motors", testModeReverse ? "Test Reverse (AndyMark Chassis)" : "Normal - Competition");
            Vector3 position = odo.getPosition();
            FlipbotSettings.storeRobotPosition(position);
            telemetry.addData("Position", position);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();

            if (intake.getHardware().robotHangMotor.getCurrent(CurrentUnit.MILLIAMPS) > 5000) {
                intake.getHardware().robotHangMotor.setPower(0);
                intake.getHardware().robotHangMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        intake.getHardware().robotHangMotor.setPower(0);  // in case it didn't get zeroed during init

        //odo.setPosition(fieldStartPos);
        robot.start();

//        hangTime = System.currentTimeMillis() + 90000;

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
//            if (hangTime != 0 && System.currentTimeMillis() > hangTime) {
//                hangTime = 0;
//                intake.tasks.prepareToHangRobotTask.restart();
//            }
            FlipbotSettings.storeRobotPosition(pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());
            telemetry.addData("Slide Position", intake.getSlidePosition());
            telemetry.addData("Lift Position", intake.getLiftPosition());
//            telemetry.addData("Speed Governor", FlipbotSettings.getControlGovernor());
            telemetry.addData("time", System.currentTimeMillis() - start);
            telemetry.addData("Last RangeDist", intake.lastRearDistance);
//            telemetry.addData("Rear Distance (inches)", intake.readRearDistance());  // this costs loop time, so remove when done
            telemetry.addData("Last SampleDist", intake.lastSampleDistance);
            telemetry.addData("Last Hue", intake.lastHue);
            telemetry.addData("Last Sample", intake.lastSample);
            telemetry.addData("Is Yellow good?", FlipbotSettings.isYellowGood);
            telemetry.addData("PIDF RUE",intake.pidf_rue);
            telemetry.addData("PIDF RTP",intake.pidf_rtp);
            telemetry.addData("SpinnerOut",intake.testSpinnerOut);


            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    public void extraSettings() {
        FlipbotSettings.isBlueGood = false;
        FlipbotSettings.isYellowGood = true;
        FlipbotSettings.isRedGood = true;
        FlipbotSettings.isDemoMode = false;
    }

    public void moveRobot(Vector3 target){
        positionSolver.setNewTarget(target, false);
    }
    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }
    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }
}
