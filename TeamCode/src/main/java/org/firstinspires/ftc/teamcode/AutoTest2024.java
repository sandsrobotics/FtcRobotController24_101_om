package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;

import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous(name="1 Josh AutoTest2024", group="Test")
public class AutoTest2024 extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    PositionSolver positionSolver;
    PositionTracker pt;
    TeamProp tp;
    Led leds;

    Vector3 startPosition;

    public void initAuto(){
        transformFunc = (v) -> v;
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    private Vector3 tileToInchAutoNoZ(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles)).withZ(tiles.Z);
    }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    @Override
    public void runOpMode() {
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot robot = new Robot(this);
        Drive drive = new Drive(robot);
        new BulkRead(robot);

        // ToDo set start position from center
        startPosition = new Vector3(2.0 * 23.5, -62, -90);
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), startPosition);
        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
        new Odometry(pt);
        pt.positionSourceId = Odometry.class;
        positionSolver = new PositionSolver(drive);
        // w/ ku = .2 and tu = 1.56, P = .12, I = .154, D = .0234
        DecimalFormat df = new DecimalFormat("#0.0");

        robot.init();

        while (!isStarted()) {}

        robot.start();

        if(shutdownps) positionSolver.triggerEvent(Robot.Events.STOP);
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);
        positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        // Here is where we schedule the tasks for the autonomous run.
        testAuto(autoTask);

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            robot.opMode.telemetry.addData("time", System.currentTimeMillis() - start);
            dashboardTelemetry.update();
            telemetry.update();
        }
        robot.stop();
    }

    private void testAuto(TimedTask autoTask) {
        Vector3 stack = new Vector3(1.0, -2.0, -90);
        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(stack), autoTask);
    }
}