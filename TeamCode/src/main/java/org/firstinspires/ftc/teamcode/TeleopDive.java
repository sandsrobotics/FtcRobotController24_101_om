package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleop;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.text.DecimalFormat;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="1 TeleopDive", group="Linear Opmode")
public class TeleopDive extends LinearOpMode {
    double tileSide = 23.5;
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
    AprilTag aprilTag;
    Vector3 fieldStartPos = new Vector3(0,0,180);
    public void initTeleop(){
        new DriveTeleop(this.drive);
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
        initTeleop();
        Led ledStick = new Led(robot);
        drive.setFieldCentric(false);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        positionSolver = new PositionSolver(drive);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
        XRelativeSolver solver = new XRelativeSolver(drive);
//        EncoderTracker et = new EncoderTracker(pt);
//        pt.positionSourceId = EncoderTracker.class;
//        Odometry24 odo = new Odometry24(pt);
//        pt.positionSourceId = Odometry24.class;
        Odometry odo = new Odometry(pt);
        pt.positionSourceId = Odometry.class;
        Intake intake = new Intake(robot);
        new IntakeTeleop(intake);
        robot.init();

        while (!isStarted()) {
            telemetry.addData("Not Started", "Not Started");
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        aprilTag = new AprilTag(robot);
        aprilTag.onInit();
        robot.start();
        aprilTag.setDesiredTag(-1);

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();

            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());

            if (aprilTag.targetFound) {
                telemetry.addData("Found", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                telemetry.addData("X", "%5.1f inches", aprilTag.desiredTag.ftcPose.x);
                telemetry.addData("Y", "%5.1f inches", aprilTag.desiredTag.ftcPose.y);
                telemetry.addData("Bearing","%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
                telemetry.addData("position: ", aprilTag.desiredTag.metadata.fieldPosition);
            }

            robot.opMode.telemetry.addData("time", System.currentTimeMillis() - start);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
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