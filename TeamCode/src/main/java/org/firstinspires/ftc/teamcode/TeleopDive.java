package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.intake2.IntakeTeleop2;
import org.firstinspires.ftc.teamcode.parts.intake2.Intake2;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import static om.self.ezftc.utils.Constants.tileSide;

@TeleOp(name="1 TeleopDive", group="Linear Opmode")
public class TeleopDive extends LinearOpMode {
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
    Vector3 fieldStartPos = new Vector3(0,0,180);
    Pinpoint odo;
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

//        startPosition = new Vector3(2.0 * 23.5, -62, -90);
//        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
//                100, new Vector3(2,2,2), startPosition);
//        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
//        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
//        odo = new Pinpoint(pt);
//        pt.positionSourceId = Pinpoint.class;
//        positionSolver = new PositionSolver(drive);
//        DecimalFormat df = new DecimalFormat("#0.0");

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false,
                100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts, PositionTrackerHardware.makeDefault(robot));
        XRelativeSolver solver = new XRelativeSolver(drive);
        odo = new Pinpoint(pt,false);
        pt.positionSourceId = Pinpoint.class;

        Intake2 intake = new Intake2(robot);
        new IntakeTeleop2(intake);
        robot.init();

        while (!isStarted()) {
            telemetry.addData("position", odo.getPosition());
            telemetry.update();
        }
        robot.start();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("Slide Position", intake.getSlidePosition());
            telemetry.addData("time", System.currentTimeMillis() - start);
            telemetry.addData("Robot lift Position", intake.getRobotLiftPosition());
            telemetry.addData("Bucket Zero", intake.getHardware().bucketLiftZeroSwitch.getState());
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
