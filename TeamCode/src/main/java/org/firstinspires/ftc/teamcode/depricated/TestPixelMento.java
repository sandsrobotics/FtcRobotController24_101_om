package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;

import java.text.DecimalFormat;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
@Disabled
@TeleOp(name="Test Pixel Mento", group="Linear Opmode")
public class TestPixelMento extends LinearOpMode {
    double tileSide = 23.5;
    boolean slideDone = false;



    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }
    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    AprilTag aprilTag;

    //Vector3 fieldStartPos = new Vector3(11.75,-63,-90);
    //Vector3 fieldStartPos = new Vector3(11.75,-63,90);
    //Vector3 fieldStartPos = new Vector3(0,0,0);
    Vector3 fieldStartPos = new Vector3(-1.5 * 23.5,-62,-90);


    public volatile TeamPropDetectionPipeline.TeamPropPosition teamPropPosition;

    @Override
    public void runOpMode() {
        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Robot robot = new Robot(this);
        new BulkRead(robot);
        Drive drive = new Drive(robot);
        new DriveTeleop(drive);
        Led ledStick = new Led(robot);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), fieldStartPos);
//        pts.withPosition(transformFunc.apply(pts.startPosition));
        //pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        //pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));
        PositionTracker pt = new PositionTracker(robot,pts,PositionTrackerHardware.makeDefault(robot));
        //PositionTracker pt = new PositionTracker(robot);

        XRelativeSolver solver = new XRelativeSolver(drive);
        //EncoderTracker et = new EncoderTracker(pt);
        //pt.positionSourceId = EncoderTracker.class;
//        Odometry24 odo = new Odometry24(pt);
//        pt.positionSourceId = Odometry24.class;
        Odometry odo = new Odometry(pt); // THIS MAY BE FIXED (warning: breaks robot lifter and sweeper cause of something with encoders)
        pt.positionSourceId = Odometry.class;
        Intake intake = new Intake(robot);
        new IntakeTeleop(intake);
        TeamProp tp = new TeamProp(robot);
        robot.init();

        while (!isStarted()) {
//            teamPropPosition = tp.pipeline.position;
//            telemetry.addData("Team Prop", teamPropPosition);
//            telemetry.addData("left:", tp.pipeline.getAvg1());
//            telemetry.addData("center:", tp.pipeline.getAvg2());
//            telemetry.addData("right:", tp.pipeline.getAvg3());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        tp.onStop(); // stop team prop detection
        aprilTag = new AprilTag(robot);
        aprilTag.onInit();
        robot.start();
        aprilTag.setDesiredTag(-1);

        // needs aprilTag and drive to initialize
        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();

            // Dashboard stuff
//            double x = pt.getCurrentPosition().X;
//            double y = pt.getCurrentPosition().Y;
//            double z = Math.toRadians(pt.getCurrentPosition().Z);
//            double x1 = Math.cos(z)*8;
//            double y1 = Math.sin(z)*8;
//            packet.fieldOverlay().setFill("blue").fillCircle(x,y,6);
//            packet.fieldOverlay().setStroke("red").strokeLine(x,y,x+x1,y+y1);

            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());

//            if(gamepad1.dpad_down) {
//                solver.setNewTarget(10, true);
//            }

//            telemetry.addData("Team Prop Position", teamPropPosition);

            if (aprilTag != null && aprilTag.targetFound) {
                telemetry.addData("Found", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                telemetry.addData("X", "%5.1f inches", aprilTag.desiredTag.ftcPose.x);
                telemetry.addData("Bearing","%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
            }

            robot.opMode.telemetry.addData("time", System.currentTimeMillis() - start);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }
}
