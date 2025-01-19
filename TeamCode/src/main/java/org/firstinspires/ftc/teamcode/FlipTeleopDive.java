package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveSettings;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleop;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="2A Flipper Teleop Arcade RED", group="Linear Opmode")
public class FlipTeleopDive extends LinearOpMode {
    double tileSide = 23.5;
    Drive drive;
    Robot robot;
    Intake intake;
    PositionSolver positionSolver;
    PositionTracker pt;
    Pinpoint odo;
  //Vector3 fieldStartPos = new Vector3(0,0,180);
    Vector3 fieldStartPos = new Vector3(-14.375,-62,90);
    boolean testModeReverse = false;

    public void initTeleop(){
        new DriveTeleop(drive, DriveTeleopSettings.makeArcade1(robot));
        //new DriveTeleop(this.drive);
    }

    @Override
    public void runOpMode() {
        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot = new Robot(this);
        new BulkRead(robot);
        drive = new Drive(robot);
//        drive = new Drive(robot, DriveSettings.makeDefault(), DriveHardware.lkTestChassis(robot.opMode.hardwareMap));
        initTeleop();

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
               100, new Vector3(2,2,2), fieldStartPos);

        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
      XRelativeSolver solver = new XRelativeSolver(drive);

//       EncoderTracker et = new EncoderTracker(pt);
//       pt.positionSourceId = EncoderTracker.class;

//        Add Pinpoint here
        odo = new Pinpoint(pt, true, "pinpoint",
                -56.0, 52.0, 13.26291192f,
                GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odo = new Pinpoint(pt);  //, true);
        pt.positionSourceId = Pinpoint.class;
        //positionSolver = new PositionSolver(drive); // removed so it won't rotate 90deg clockwise

        intake = new Intake(robot);
        new IntakeTeleop(intake);

        robot.init();
        odo.setPosition(fieldStartPos);

        extraSettings();

        while (!isStarted()) {
            robot.buttonMgr.runLoop();
            telemetry.addData("Not Started", "Not Started");
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasDoubleTapped)) {
                drive.lkUpdateConfig(DriveSettings.makeDefault(), DriveHardware.lkTestChassis(robot.opMode.hardwareMap));
                testModeReverse = true;
            }
            if (robot.buttonMgr.getState(1, ButtonMgr.Buttons.x, ButtonMgr.State.wasSingleTapped)) {
                drive.lkUpdateConfig(DriveSettings.makeDefault(), DriveHardware.makeDefault(robot.opMode.hardwareMap));
                testModeReverse = false;
            }
            telemetry.addData("Drive motors", testModeReverse ? "Test Reverse (AndyMark Chassis)" : "Normal - Competition");
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        odo.setPosition(fieldStartPos);
        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());
            telemetry.addData("Slide Position", intake.getSlidePosition());
            telemetry.addData("Lift Position", intake.getLiftPosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    public void extraSettings() {
        intake.isBlueGood = false;
        intake.isYellowGood = true;
        intake.isRedGood = true;
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
