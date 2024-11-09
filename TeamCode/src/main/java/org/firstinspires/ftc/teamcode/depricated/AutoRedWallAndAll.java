package org.firstinspires.ftc.teamcode.depricated;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;

import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Disabled
@Autonomous(name="1 RED-WALL", group="Test")
public class AutoRedWallAndAll extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    Intake intake;
    PositionSolver positionSolver;
    PositionTracker pt;
    EncoderTracker et;
    public AprilTag aprilTag;
    TeamProp tp;
    Led leds;
    public boolean center, left, right, lefttagleft, lefttagright, centertagleft, centertagright, righttagleft, righttagright;
    public boolean midPark;
    public boolean isRed;
    public boolean parkOnly;
    public boolean isBoard;
    public boolean extraPix;
    public boolean stackPathSide;
    public boolean dropPathSide;
    public boolean extraWallPix;
    public boolean dropLow;
    public boolean stackSide;
    public boolean purple;
    private boolean retryStack;
//  DASHBOARD VARIABLES (static public)
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    static public double P = 0.05;
    static public double I;
    static public double D;
    static public double angleP = 0.0125;
    static public double angleI;
    static public double angleD;
    static public Vector3 testPos = new Vector3(1.5, -1, -180);
    Vector3 stack = new Vector3(-2.38, -0.55, 180);
//  ********************************
    public int startDelay;
    private int parkPosition;
    private int pixels;
    private TeamPropDetectionPipeline.PixelPosition pixPos;
    private final double stackDist = 58.5 - 7.3125;
    private final double stack2Dist = 35 - 7.3125; // was 35.25 (1.5 * 23.5)
    private final double lLeft = 44 - 7.3125;
    private final double lRight = lLeft - 3.0;
    private final double cLeft = lRight - 3.0;
    private final double cRight = cLeft - 3.1;
    private final double rLeft = cRight - 3.0;
    private final double rRight = rLeft - 3.0;
    private double stackY;

    //Vector3 centralspikemark = new Vector3(-35.25, -39.5, -90);
   // Vector3 startPosition = new Vector3(-1.5,-2.7,-90);
    Vector3 startPosition;

    public void initAuto(){
        transformFunc = (v) -> v;
        isRed = true;
        midPark = true;
        parkOnly = false;
        isBoard = false;
        extraPix = true;
        dropLow = false;
        stackPathSide = false;
        dropPathSide = false;
        extraWallPix = false;
        stackSide = false;
        retryStack = true;
        purple = false;
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
        Robot r = new Robot(this);
        Drive d = new Drive(r);
        new BulkRead(r);
        tp = new TeamProp(r);
        intake = new Intake(r);
        leds = new Led(r);

//        if(isRed)
//            intake.blueSideDist = intake.getHardware().blueSensor.getDistance(DistanceUnit.INCH);
//        else
//            intake.redSideDist = intake.getHardware().redSensor.getDistance(DistanceUnit.INCH);
//
//        startPosition = new Vector3(-(70.5-((isRed ? intake.getBlueSideDist() : intake.getRedSideDist()) + 7.3125)),-62,-90);
//        if(!(startPosition.X <= -1.5 * 23.5 + 3 && startPosition.X >= -1.5 * 23.5 - 3))
            startPosition = new Vector3(-1.5 *23.5, -62, -90);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), startPosition);
//            Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90); // red wall
//        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
//         customStartPos = new Vector3(-1.5 * 23.5,-62,90); // blue wall side

        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));
        Odometry odo = new Odometry(pt);
        pt.positionSourceId = Odometry.class;
//        positionSolver = new PositionSolver(d, new PositionSolverSettings(
//                new SolverSettings(1, 10, true, 10000, new PIDCoefficients(P, I, D), 1),
//                new SolverSettings(1, 10, true, 10000, new PIDCoefficients(P, I, D), 1),
//                new SolverSettings(2.5, 10, true, 10000, new PIDCoefficients(angleP, angleI, angleD), 1)
//        ));
        positionSolver = new PositionSolver(d);
        // w/ ku = .2 and tu = 1.56, P = .12, I = .154, D = .0234
        DecimalFormat df = new DecimalFormat("#0.0");
        r.init();
        int maxDelay = isBoard ? 18000 : 6000;

        while (!isStarted()) {
            if (tp.pipeline.position== TeamPropDetectionPipeline.TeamPropPosition.NONE)
                telemetry.addData("***** WAIT *****\n***** WAIT *****\n***** WAIT *****", "");
            else
                telemetry.addData("Team Prop", tp.pipeline.position);

            if(new EdgeSupplier(()-> r.opMode.gamepad1.right_bumper).isRisingEdge()) {
                startDelay += 1000;
            }
            else if(new EdgeSupplier(()->r.opMode.gamepad1.left_bumper).isRisingEdge()) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.a).isRisingEdge()) {
                parkPosition = 1;
                maxDelay = isBoard ? 18000 : 6000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.b).isRisingEdge()) {
                parkPosition = 2;
                maxDelay = isBoard ? 18000 : 6000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.x).isRisingEdge()) {
                parkPosition = 3;
                maxDelay = isBoard ? 20000 : 7000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.y).isRisingEdge()) {
                parkPosition = 0;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_up).isRisingEdge()) {
                extraPix = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_down).isRisingEdge()) {
                extraPix = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_left).isRisingEdge()){
                dropLow = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_right).isRisingEdge()){
                dropLow = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.dpad_left).isRisingEdge()){
                stackPathSide = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.dpad_right).isRisingEdge()){
                stackPathSide = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.dpad_down).isRisingEdge()){
                dropPathSide = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.dpad_up).isRisingEdge()){
                dropPathSide = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.x).isRisingEdge()){
                extraWallPix = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.y).isRisingEdge()){
                extraWallPix = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.right_bumper).isRisingEdge()){
                stackSide = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad2.left_bumper).isRisingEdge()){
                stackSide = false;
            }


            if(startDelay > maxDelay) startDelay = maxDelay;

//            telemetry.addData("START POSITION: ", startPosition);
//            telemetry.addData("TEAMPROP Pixel pos", tp.pixPos);
//            telemetry.addData("PIPELINE Pixel pos", tp.pipeline.pixelPosition);
//            telemetry.addData("PIPELINE RETURNED Pixel pos", tp.pipeline.getPixAnalysis());
            telemetry.addData("BOARD TO STACK PATHING: ", dropPathSide ? "SIDE" : "MID");
            telemetry.addData("STACK TO BOARD PATHING: ", stackPathSide ? "SIDE" : "MID");
            telemetry.addData("STACK: ", stackSide ? "SIDE" : "MID");
            telemetry.addData("EXTRA PIXEL? ", extraPix);
            if(!isBoard)
                telemetry.addData("EXTRA STACK?", extraWallPix);
            telemetry.addData("DROP POSITION: ", dropLow ? "Drop first" : "Drop second");
            telemetry.addData("PARK POSITION:", parkPosition == 0 ? "Park based off tags" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
            telemetry.addData("START DELAY:", startDelay / 1000);
            telemetry.addData("AUTO: ", isRed ? (isBoard ? "RED BOARD" : "RED WALL") : isBoard ? "BLUE BOARD" : "BLUE WALL");
            telemetry.update();
            sleep(50);
        }

//        tp.onStop();
//        aprilTag = new AprilTag(r);
//        aprilTag.onInit();
        r.start();

        center = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.CENTER);
        left = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.LEFT);
        right = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.RIGHT);


        if(shutdownps)
            positionSolver.triggerEvent(Robot.Events.STOP);

        if(isRed) {
//            parkPosition = 1;
//            aprilTag.setDesiredTag(center ? 5 : left ? 4 : 6);
        }
        else {
            if(left) {
                left = false;
                right = true;
            }
            else if(right) {
                right = false;
                left = true;
            }
//            aprilTag.setDesiredTag(center ? 2 : left ? 1 : 3);
        }

        Group container = new Group("container", r.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);

        Group checker = new Group("checker", r.taskManager);
        checker.autoStartPolicy = Group.AutoManagePolicy.DISABLED;
        TimedTask checkTask = new TimedTask("check task", checker);

        positionSolver.setNewTarget(pt.getCurrentPosition(), true);


        checkTask.addStep(()->{
            if(pixels == 2) {
                dropOneAuto(autoTask);
                parkAuto(autoTask);
            }
            else if(pixels == 1) {
                dropOneAuto(autoTask);
                parkAuto(autoTask);
            }
//            else if(pixels == 0 && retryStack) {
//                retryStack(autoTask, stack);
//                autoTask.addStep(checkTask::restart);
//            }
            else
                nopixPark(autoTask);
//            if(!retryStack)
        });

//        autoTask.addTimedStep(()->intake.sweepWithPower(1), 2000);
        autoTask.addStep(() -> {
            // init intake setup items here
        });
        // add calls to special autonomous action collections in methods below
        if(!parkOnly) {
            autoTask.addDelay(startDelay);
            if (isBoard) {
                if(extraPix) {
                    boardAutoGrabPix(autoTask);
                    autoTask.addStep(checker);
                }
                else{
                    boardAuto(autoTask);
                    dropAuto(autoTask);
                    parkAuto(autoTask);
                }
            }
            else {
                if(extraPix && extraWallPix) {
                    wallAutoGrabPix(autoTask);
                    autoTask.addStep(checker);
                } else if (extraPix){
                    wallAutoGrabPix(autoTask);
                    parkAuto(autoTask);
                }
                else if(purple)
                    wallAuto(autoTask);
                else {
                    wallAuto(autoTask);
                    dropAuto(autoTask);
                    parkAuto(autoTask);
                }
            }
//            parkAuto(autoTask);
        } else {
            testAuto(autoTask);
        }

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            telemetry.addData("Team Prop Position", tp.pipeline.position);
//            double x = pt.getCurrentPosition().X;
//            double y = pt.getCurrentPosition().Y;
//            double z = Math.toRadians(pt.getCurrentPosition().Z);
//
//            double x1 = Math.cos(z)*8;
//            double y1 = Math.sin(z)*8;
//            packet.fieldOverlay().setFill("blue").fillCircle(x,y,6);
//            packet.fieldOverlay().setStroke("red").strokeLine(x,y,x+x1,y+y1);
//            packet.fieldOverlay().fillCircle(1, 1, 1);

            r.run();
            //container.run();
            dashboardTelemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("expected Y position: ", stackY);
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("PIPELINE Pixel pos", tp.pixPos);
            telemetry.addData("!!!!!AUTO LOCKED Pixel pos!!!!!!", pixPos);
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
//            telemetry.addData("target found?", aprilTag.targetFound);
//            if (aprilTag.targetFound) {
//                telemetry.addData("Found", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
//                telemetry.addData("X", "%5.1f inches", aprilTag.desiredTag.ftcPose.x);
//                telemetry.addData("Bearing","%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw","%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
//            } else {
//                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//            }
            r.opMode.telemetry.addData("time", System.currentTimeMillis() - start);
            dashboardTelemetry.update();
            telemetry.update();

        }
        r.stop();
    }

    private void parkAuto(TimedTask autoTask){
        Vector3 parkMid = new Vector3(2.4, -.48, 180);
        Vector3 parkSide = new Vector3(2.4, -2.43, 180);
        Vector3 preParkMid = new Vector3(2.0, -.5, 180);
        Vector3 preParkSide = new Vector3(2.0, -2.5, 180);
        Vector3 centerAT = new Vector3(1.8,-1.55,180);
        Vector3 board = new Vector3(1.8,-1.55,180);

//        if(!(parkPosition == 3 || parkPosition == 2))
//            positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        if(parkPosition == 0) {
            intake.addAutoDockToTask(autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(left ? preParkSide : preParkMid), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(left ? parkSide : parkMid), autoTask);
        }
        else if(parkPosition == 3){
            positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
            intake.addAutoDockToTask(autoTask);
        }
        else{
            intake.addAutoDockToTask(autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(parkPosition == 1 ? preParkMid : preParkSide), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(parkPosition == 1 ? parkMid : parkSide), autoTask);
        }
//        autoTask.addStep(()->intake.setSweepPosition(1));
//        autoTask.addStep(()->intake.setSweepPosition(4)); // for easier autonomous setup
    }

    private void dropAuto(TimedTask autoTask) {
        Vector3 centerAT = new Vector3(1.5, -1.55, 180);
        Vector3 leftAT = new Vector3(1.5, -1.22, 180);
        Vector3 rightAT = new Vector3(1.5, -1.72, 180);
        Vector3 scenterAT = new Vector3(1.8, -1.55, 180);
        Vector3 sleftAT = new Vector3(1.8, -1.27, 180);
        Vector3 srightAT = new Vector3(1.8, -1.72, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 postStack = new Vector3(-2.4, stackPathSide ? -2.5 : -.5, 180);
        Vector3 beforeRig = new Vector3(-1.5, -1.5, 90);
        Vector3 throughRigging = new Vector3(-2.4, -2.5, 180);
        Vector3 throughRigging2 = new Vector3(-1.5, -2.5, 90);
        Vector3 throughRigging2turn = new Vector3(-1.5, -2.5, 180);
        Vector3 pixCheck = new Vector3(1.5, -1.5, 180);

        if (!isBoard && extraPix) {
            if (stackSide)
                positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
            if (stackPathSide) {
                positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
                autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
                positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
            } else {
                autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
                positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
            }
        } else if (!isBoard){
            if(stackPathSide) {
                positionSolver.addMoveToTaskEx(tileToInchAuto(beforeRig), autoTask);
                positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging2), autoTask);
                positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging2turn), autoTask);
                autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
                positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
            }
            else{
                autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
                positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
            }
        }

        if(!isBoard)
            intake.addAutoDropToTask(autoTask);
//        autoTask.addStep(() -> intake.setGrabPosition(3));
//        if (isRed){
//            if(right||!isBoard)
//                intake.addAutoDropToTask(autoTask);
//        } else {
//            if(!center && !right && !left)
//                intake.addAutoDropToTask(autoTask);
//        }
//        if(!isBoard) {
        if(!isBoard) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pixCheck), autoTask);
            autoTask.addDelay(2000); // lock position to detect the pixel better
            autoTask.addStep(() -> {
                pixPos = tp.pipeline.getPixAnalysis();
                if(isRed) {
                    intake.ulSideDist =
                            pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGLEFT && right ? rRight :
                                    pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGRIGHT && left ? lLeft :
                                            pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGLEFT && left ? lRight :
                                                    pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGRIGHT && right ? rLeft :
                                                            pixPos == TeamPropDetectionPipeline.PixelPosition.CENTERTAGLEFT && center ? cRight :
                                                                    pixPos == TeamPropDetectionPipeline.PixelPosition.CENTERTAGRIGHT && center ? cLeft : 100000;
                }
                else {
                    intake.ulSideDist =
                            pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGLEFT && left ? rRight :
                                    pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGRIGHT && right ? lLeft :
                                            pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGLEFT && right ? lRight :
                                                    pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGRIGHT && left ? rLeft :
                                                            pixPos == TeamPropDetectionPipeline.PixelPosition.CENTERTAGLEFT && center ? cRight :
                                                                    pixPos == TeamPropDetectionPipeline.PixelPosition.CENTERTAGRIGHT && center ? cLeft : 100000;

                }
            });
            autoTask.addStep(() -> {
                if(isRed)
                    intake.sideRangeDist = intake.ulSideDist;
                else
                    intake.sideRangeDist = 3*23.5 - intake.ulSideDist;
            });
//        autoTask.addDelay(3000);
        }
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? centerAT : left ? leftAT : rightAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));

        if(!isBoard){
            autoTask.addConditionalDelay(400, ()-> !dropLow); //give swing arm time to get out before lowering slider
            autoTask.addStep(()-> intake.setSlidePosition(dropLow ? 650 : 1000));
            autoTask.addStep(() -> {
                intake.run = true;
                if(isRed) intake.runRed = true;
                else intake.runBlue = true;
            });
            autoTask.addConditionalDelay(5000, ()->!intake.getHardware().grabberLimitSwitch.getState());
            autoTask.addStep(()-> {
                intake.run = false;
                intake.runRed = false;
                intake.runBlue = false;
            });
            autoTask.addStep(()->intake.setGrabPosition(extraPix ? 2 : 1));
            autoTask.addDelay(400);
            intake.addFoundRangeToTask(autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? sleftAT : scenterAT), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
            autoTask.addConditionalDelay(750, ()->!extraPix);
            if(dropLow)
                autoTask.addStep(()-> intake.setSlidePosition(1000));
//            autoTask.addDelay(500); //magic deelay *(DO NORT MREOVE)
        } else{
            autoTask.addDelay(500); //give swing arm time to get out before lowering slider (ALSO MAGICAL)
            autoTask.addStep(()-> intake.setSlidePosition(dropLow ? 700 : 1000));
            autoTask.addDelay(750); //magic deelay *(DO NORT MREOVE)
        }
        if(!isBoard && extraPix) {
            autoTask.addDelay(750); //magic deelay *(DO NORT MREOVE)
            autoTask.addStep((Runnable) () -> intake.run = true);
            autoTask.addConditionalDelay(3000, () -> !intake.getHardware().grabberLimitSwitch.getState());
            autoTask.addStep((Runnable) () -> intake.run = false);
            autoTask.addStep(() -> intake.setGrabPosition(1));
            autoTask.addConditionalDelay(100, ()->extraPix = false);
            intake.addFoundRangeToTask(autoTask);
        } else if(isBoard){
            autoTask.addStep((Runnable) () -> intake.run = true);
            autoTask.addConditionalDelay(7000, ()->!intake.getHardware().grabberLimitSwitch.getState());
            autoTask.addStep((Runnable) ()->intake.run = false);
            autoTask.addStep(()->intake.setGrabPosition(1));
            autoTask.addDelay(100);
            intake.addFoundRangeToTask(autoTask);
        }
//        autoTask.addDelay(250); //make sure pixel is dropped before pulling away
        autoTask.addStep(()-> intake.setSlidePosition(1000));
        autoTask.addDelay(250);
//        autoTask.addDelay(500); //give slider time to get up otherwise it wont dock properly and crash on park
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
    }

    private void wallAuto(TimedTask autoTask) {
        Vector3 startPos = new Vector3(-1.5, -2.6, -90);
        Vector3 pushProp = new Vector3(-1.5, -1.38, -90);
        Vector3 centerRight = new Vector3(-1.7, -1.38, 0);
        Vector3 centerP = new Vector3(-1.7, -1.5, -90);
        Vector3 centerTurn = new Vector3(-1.7, -1.5, 90);
        Vector3 moveRight = new Vector3(-1.7, -.5, 0);
        Vector3 dropCenter = new Vector3(-1.5, -.6, -90);
        Vector3 dropPixLeft = new Vector3(-1.49, -1.38, 180);
        Vector3 postDropPixLeft = new Vector3(-1.5, -1.38, 180);
        Vector3 preDropPixRight = new Vector3(-1.47, -1.38, -90);
        Vector3 dropPixRight = new Vector3(-1.47, -1.38, 0);
        Vector3 preSetupTagsMid = new Vector3(-1.5, -.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 preSetupTagsMidRIGHT = new Vector3(-1.7, -.5, 180);
        Vector3 stack = new Vector3(-2.5, -.5, 180);
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 postStack = new Vector3(-2.2, stackPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-2, -2.5, 180);


        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(()->intake.setGrabPosition(3));
        if (center && stackPathSide){
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerP), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerTurn), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
//            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        } else if (center){
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropCenter), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 750);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
        }
        else if(left){
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
//            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        } else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDropPixRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixRight), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
//            positionSolver.addMoveToTaskEx(tileToInchAuto(centerRight),autoTask);
//            positionSolver.addMoveToTaskEx(tileToInchAuto(moveRight), autoTask);
//            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMidRIGHT), autoTask);
        }
//        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
    }

    private void boardAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(.5, -2.6, 90);
        Vector3 preDrop = new Vector3(.5, -1.38, -90);
        Vector3 dropPixCenter = new Vector3(.5, -.62, -90);
        Vector3 postCenter = new Vector3(1, 0, -90);
        Vector3 dropPixLeft = new Vector3(.5, -1.38, 180);
        Vector3 setupTags = new Vector3(1.5, -1.5, 180);
        Vector3 turnRight = new Vector3(1.37, -1.5, 180);
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 postStack = new Vector3(-2.2, stackPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-2, -2.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(()->intake.setGrabPosition(3));
        if(center){
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixCenter), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 600);
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(postCenter), autoTask);
        }
        else if (left) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDrop), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
        }
        else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(turnRight), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
            intake.addAutoDropToTask(autoTask);
        }
        autoTask.addDelay(250);
        if(center || left) {
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(setupTags), autoTask);
            autoTask.addDelay(center ? 500 : 20);
            intake.addAutoDropToTask(autoTask);
        }
        else
            positionSolver.addMoveToTaskEx(tileToInchAuto(setupTags), autoTask);
    }

    private void boardAutoGrabPix(TimedTask autoTask){
        Vector3 postTag = new Vector3(1.3, dropPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-2, -2.5, 180);
        Vector3 preStack = new Vector3(-2.3, stackSide ? -1.5 : -.5, 180);
        Vector3 preStackAvoidLeft = new Vector3(-2.3, -2, 180);
        Vector3 stack = new Vector3(-2.4, -.5, 180);
        Vector3 throughMid = new Vector3(-2, -.5, 180);
        Vector3 stack2 = new Vector3(-2.425, -1.52, 180);
        Vector3 preSecondStack = new Vector3(-2.35, -1.5, 180);

        boardAuto(autoTask);
        dropAuto(autoTask);

//        autoTask.addDelay(1000);
        intake.addAutoDockToTask(autoTask);
//        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        if(dropPathSide) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
            if (stackSide) {
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preStackAvoidLeft), autoTask);
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                }
            else {
                positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
            }
        }
        else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(throughMid), autoTask);
            if (stackSide) {
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preStackAvoidLeft), autoTask);
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                }
            else {
                positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
            }
        }
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));

        grabFromStackAuto(autoTask, 2, stackSide ? stack2 : stack);
    }

    private void wallAutoGrabPix(TimedTask autoTask) {
    Vector3 startPos = new Vector3(-1.5, -2.6, -90);
    Vector3 pushProp = new Vector3(-1.5, -1.38, -90);
    Vector3 centerRight = new Vector3(-1.7, -1.38, 0);
    Vector3 moveRight = new Vector3(-1.7, -.5, 0);
    Vector3 dropCenter = new Vector3(-1.5, -.6, -90);
    Vector3 dropPixLeft = new Vector3(-1.5, -1.38, 180);
    Vector3 postDropPixLeft = new Vector3(-1.5, -1.38, 180);
    Vector3 preDropPixRight = new Vector3(-1.47, -1.38, -90);
    Vector3 dropPixRight = new Vector3(-1.5, -1.38, 0);
    Vector3 preSetupTagsMid = new Vector3(-1.5, -.5, 180);
    Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
    Vector3 preStack = new Vector3(-2.4, -.5, 180);
    Vector3 stack = new Vector3(-2.43, -.55, 180);
    Vector3 stackBlue = new Vector3(-2.37, -.5, 180);
    Vector3 postTag = new Vector3(1, dropPathSide ? -2.5 : -.5, 180);
    Vector3 throughRigging = new Vector3(-2.4, -2.5, 180);
    Vector3 preSecondStack = new Vector3(-2.4, -1.5, 180);
    Vector3 preStackAvoidLeft = new Vector3(-2.4, -2, 180);
    Vector3 stack2 = new Vector3(-2.42, -1.53, 180);
    Vector3 throughMid = new Vector3(-2, -.5, 180);


        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(() -> intake.setGrabPosition(1));
        if (center) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropCenter), autoTask);
//            autoTask.addDelay(1000);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 500);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));

        } else if (left) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            if(!isRed)
                positionSolver.addMoveToTaskEx(tileToInchAuto(postDropPixLeft), autoTask);
        } else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDropPixRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixRight), autoTask);
            autoTask.addTimedStep(()->intake.sweepWithPower(1), 1000);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(moveRight), autoTask);
        }
        positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        if(stackSide) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
        }
        autoTask.addStep(() -> intake.setSweepPosition(3));
        intake.addAutoGrabToTask(autoTask, false);
        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
//        autoTask.addDelay(2000);
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(stackSide ? stack2 : stack), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowNoAlwaysRunSettings));
        autoTask.addDelay(750);
        autoTask.addStep(()->{
            intake.sideRangeDist = stackSide ? stack2Dist : stackDist;
            if(isRed) intake.runRed = true;
            else intake.runBlue = true;
        });
        if(isRed)
            autoTask.addConditionalDelay(1500, ()-> intake.runRed = intake.hasPixels() == 2);
        else
            autoTask.addConditionalDelay(1500, ()-> intake.runBlue = intake.hasPixels() == 2);
        autoTask.addStep(()->{
            if(isRed) intake.runRed = false;
            else intake.runBlue = false;
        });
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 1500);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 1500);
//        autoTask.addTimedStep(()->intake.sweepWithPower(1), 500); // dump any extra pixels
        intake.addAutoGrabToTask(autoTask, true);
        autoTask.addStep(()-> setExtraPix(intake.hasPixels() == 2));
//        autoTask.addStep(()-> pixels = intake.hasPixels());
        autoTask.addStep((Runnable) ()-> intake.extraDrop = extraPix);

        dropAuto(autoTask);

        if(extraWallPix) {
            intake.addAutoDockToTask(autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            if(dropPathSide) {
                positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
                if (stackSide) {
                    if (right)
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                    else if (dropPathSide) {
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preStackAvoidLeft), autoTask);
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                    }
                } else {
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
                }
            }
            else {
                positionSolver.addMoveToTaskEx(tileToInchAuto(throughMid), autoTask);
                if (stackSide) {
                    if (right)
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                    else if (dropPathSide) {
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preStackAvoidLeft), autoTask);
                        positionSolver.addMoveToTaskEx(tileToInchAuto(preSecondStack), autoTask);
                    }
                } else {
                    positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
                }
            }
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));

            grabFromStackAuto(autoTask, 3, stackSide ? stack2 : stack);
        }

    }

    public void grabFromStackAuto(TimedTask autoTask, int height, Vector3 stack){
        Vector3 postStack = new Vector3(-2.3, -.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 preStack = new Vector3(stack.X+0.1, stack.Y, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.extraLoseSettings));
        autoTask.addStep(() -> intake.setSweepPosition(height));
//        intake.addAutoGrabToTask(autoTask, false); // pickup from white stack
        positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
        intake.addAutoGrabToTask(autoTask, false);
        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
//        autoTask.addDelay(500);
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(stack), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowNoAlwaysRunSettings));
//        autoTask.addDelay(500);
        autoTask.addStep(()->{
            intake.sideRangeDist = stackSide ? stack2Dist : stackDist;
            if(isRed) intake.runRed = true;
            else intake.runBlue = true;
        });
        if(isRed)
            autoTask.addConditionalDelay(3000, ()-> intake.runRed = intake.hasPixels() == 2);
        else
            autoTask.addConditionalDelay(3000, ()-> intake.runBlue = intake.hasPixels() == 2);
        autoTask.addStep(()->{
            intake.runRed = false;
            intake.runBlue = false;
        });
        //three checks that we have 2 pixels since sometimes it accidentally sees 2 when grabbing 1
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2 || intake.hasPixels() == 1, 1000);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 200);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 200);
        autoTask.addStep(()->intake.setSweepPosition(4));
        autoTask.addTimedStep(()->intake.sweepWithPower(1), 750); // dump any extra pixels
        autoTask.addStep((Runnable) ()->intake.runRed = false);
        intake.addAutoGrabToTask(autoTask, true);
        autoTask.addStep(()-> setExtraPix(intake.hasPixels() == 2));
        autoTask.addStep(()-> pixels = intake.hasPixels());
        autoTask.addStep((Runnable) ()-> intake.extraDrop = extraPix);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
    }

    private void dropTwoAuto(TimedTask autoTask) {
        Vector3 centerAT = new Vector3(1.5,-1.55,180);
        Vector3 leftAT = new Vector3(1.5, -1.23, 180);
        Vector3 rightAT = new Vector3(1.5, -1.82, 180);
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 postStack = new Vector3(-2.4, stackPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-2.3, -2.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 scenterAT = new Vector3(1.8,-1.55,180);
        Vector3 sleftAT = new Vector3(1.8, -1.27, 180);

//        autoTask.addStep(()->intake.setSweepPosition(4));
        positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
//        autoTask.addStep(()->intake.setSweepPosition(4));
        if(stackPathSide){
            positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
        }
        else{
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
        }
        autoTask.addStep(() -> intake.setGrabPosition(3));
        intake.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? rightAT : centerAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        autoTask.addDelay(500); //give swing arm time to get out before lowering slider
        autoTask.addStep(()-> intake.setSlidePosition(1600));
        autoTask.addStep((Runnable) () -> intake.run = true);
        autoTask.addConditionalDelay(5000, ()->!intake.getHardware().grabberLimitSwitch.getState());
        autoTask.addStep((Runnable) ()->intake.run = false);
        autoTask.addStep(()->intake.setGrabPosition(2));
        autoTask.addDelay(250);
        intake.addFoundRangeToTask(autoTask);
//        autoTask.addDelay(250); //magic deelay *(DO NORT MREOVE)
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
//        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? sleftAT : scenterAT), autoTask);
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        autoTask.addDelay(500); //magic deelay *(DO NORT MREOVE)
        autoTask.addStep((Runnable) () -> intake.run = true);
        autoTask.addConditionalDelay(5000, ()->!intake.getHardware().grabberLimitSwitch.getState());
        autoTask.addStep((Runnable) ()->intake.run = false);
        autoTask.addStep(()->intake.setGrabPosition(1));
        autoTask.addDelay(250); //make sure pixel is dropped before pulling away
        intake.addFoundRangeToTask(autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
    }

    private void dropOneAuto(TimedTask autoTask) {
        Vector3 centerAT = new Vector3(1.7,-1.55,180);
        Vector3 leftAT = new Vector3(1.7, -1.23, 180);
        Vector3 rightAT = new Vector3(1.7, -1.82, 180);
        Vector3 postTag = new Vector3(1.5, -2.5, 180);
        Vector3 postStack = new Vector3(-2.2, stackPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-1.5, -2.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 sleftAT = new Vector3(1.8, -1.27, 180);

        if(!stackSide)
            positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
        if(stackPathSide){
            positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
        }
        else{
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
        }
        intake.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(sleftAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
//        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? (isRed ? leftAT : rightAT) : centerAT), autoTask);
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(sleftAT), autoTask); // TAKE THIS OUT AFTER MATCH 110!!!!
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        autoTask.addDelay(300); //give swing arm time to get out before lowering slider
        autoTask.addStep(()-> intake.setSlidePosition(1600));
        autoTask.addStep((Runnable) () -> intake.run = true);
        autoTask.addConditionalDelay(7000, ()->!intake.getHardware().grabberLimitSwitch.getState());
        autoTask.addStep((Runnable) ()->intake.run = false);
        autoTask.addStep(()->intake.setGrabPosition(2));
        autoTask.addDelay(750);
        autoTask.addStep(()->intake.setGrabPosition(1));
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
    }

    private void retryStack(TimedTask autoTask, Vector3 stack){
//        Vector3 stack = new Vector3(-2.38, -.5, 180);
        Vector3 stack2 = new Vector3(-2.38, -1.5, 180);
        Vector3 leftStack = new Vector3(stack.X - .1, stack.Y - .2, 180);
        Vector3 rightStack = new Vector3(stack.X - .1, stack.Y + .2, 180);
        Vector3 postStack = new Vector3(-2.2, stackPathSide ? -2.5 : -.5, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(()->intake.setSweepPosition(1));
        autoTask.addStep(()->intake.setGrabPosition(1));
        autoTask.addStep((Runnable) ()->retryStack = false);
        intake.addAutoGrabToTask(autoTask);
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(leftStack), autoTask);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 2500);
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(rightStack), autoTask);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 2500);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
        autoTask.addTimedStep(()->intake.sweepWithPower(1), 750); // dump any extra pixels
        autoTask.addStep(() -> intake.setGrabPosition(3));
        autoTask.addStep(()-> setExtraPix(intake.hasPixels() == 2));
        autoTask.addStep(()-> pixels = intake.hasPixels());
        autoTask.addStep((Runnable) ()-> intake.extraDrop = extraPix);
        autoTask.addDelay(5000);
    }


        private void testAuto(TimedTask autoTask) {
        Vector3 stack = new Vector3(1.5, -1.5, 180);
        Vector3 postRange = new Vector3(2.0, 0.5, -180);

//        autoTask.addStep();        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
//        autoTask.addStep(()->pixPos = tp.pipeline.getPixAnalysis());
//        autoTask.addStep(()->{intake.sideRangeDist =
//                pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGLEFT ? lRight :
//                        pixPos == TeamPropDetectionPipeline.PixelPosition.LEFTTAGRIGHT ? lLeft :
//                                pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGLEFT ? rRight :
//                                        pixPos == TeamPropDetectionPipeline.PixelPosition.RIGHTTAGRIGHT ? rLeft :
//                                                pixPos == TeamPropDetectionPipeline.PixelPosition.CENTERTAGLEFT ? cRight : cLeft;
//        });
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.slowSettings));
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(stack), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowNoAlwaysRunSettings));
            autoTask.addStep(()->{
                intake.sideRangeDist = stackSide ? stack2Dist : stackDist;
                intake.runRed = true;
            });
                autoTask.addConditionalDelay(8000, ()-> intake.runRed = intake.hasPixels() == 2);
//            else
//                autoTask.addConditionalDelay(3000, ()-> intake.runBlue = intake.hasPixels() == 2);
            autoTask.addStep(()->{
                intake.runRed = false;
                intake.runBlue = false;
            });
        }

    private void nopixPark(TimedTask autoTask){
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 postStack = new Vector3(-2.2, stackPathSide ? -2.5 : -.5, 180);
        Vector3 throughRigging = new Vector3(-1.5, -2.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 board = new Vector3(1.8,-1.55,180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
        if(stackPathSide){
            positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
        }
        else{
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
        }
        if(parkPosition == 3)
            positionSolver.addMoveToTaskEx(tileToInchAuto(board), autoTask);
        parkAuto(autoTask);
    }

    public void setExtraPix(boolean extraPix) {
        this.extraPix = extraPix;
    }
}