package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleop;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="Flipper Teleop", group="Linear Opmode")
public class FlipTeleopDive extends LinearOpMode {
    double tileSide = 23.5;
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
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

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        XRelativeSolver solver = new XRelativeSolver(drive);

        EncoderTracker et = new EncoderTracker(pt);
        pt.positionSourceId = EncoderTracker.class;

//        Add Pinpoint here
//        Pinpoint odo = new Pinpoint(robot);
//        pt.positionSourceId = Pinpoint.class;

        Intake intake = new Intake(robot);
        new IntakeTeleop(intake);

        robot.init();

        while (!isStarted()) {
            telemetry.addData("Not Started", "Not Started");
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());
            telemetry.addData("Slide Position", intake.getSlidePosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
            //telemetry.addData("Robot lift Position", intake.getRobotLiftPosition());
            //telemetry.addData("Current Bucket Position", intake.getBucketLiftPosition());
            telemetry.addData("Current Vertical Slide Pos:", intake.getSettings().v_Slide_pos);
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
