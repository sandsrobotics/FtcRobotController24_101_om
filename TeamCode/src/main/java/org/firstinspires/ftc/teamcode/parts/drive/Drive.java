package org.firstinspires.ftc.teamcode.parts.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.parts.drive.hardware.DriveHardware;
import org.firstinspires.ftc.teamcode.parts.drive.settings.DriveSettings;

import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllableLoopedPart;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.modifiers.SimpleRampedModifier;

public final class Drive extends ControllableLoopedPart<Robot,DriveSettings, DriveHardware, DriveControl> {
    private Vector3 targetPower;

    private final SimpleRampedModifier xRamp = new SimpleRampedModifier();
    private final SimpleRampedModifier yRamp = new SimpleRampedModifier();
    private final SimpleRampedModifier rRamp = new SimpleRampedModifier();

    private Function<Vector3, Vector3> powerFilter = pow -> pow;

    private DrivePowerConverter dpc;
    private boolean fieldCentric = false;

    public  Drive(Robot robot){
        super(robot, "drive", robot.regularTaskManager, () -> new DriveControl(new Vector3(0,0,0),false));
        setConfig(
                DriveSettings.makeDefault(),
                DriveHardware.makeDefault(robot.opMode.hardwareMap)
        );

    }

    public Drive(Robot robot, DriveSettings driveSettings, DriveHardware driveHardware) {
        super(robot, "drive", () -> new DriveControl(new Vector3(0,0,0),false));
        setConfig(driveSettings, driveHardware);
    }

    public Vector3 getTargetPower(){
        return targetPower;
    }

    public int[] getMotorPositions(){
        return new int[]{
            getHardware().topLeftMotor.getCurrentPosition(),
                getHardware().topRightMotor.getCurrentPosition(),
                getHardware().bottomLeftMotor.getCurrentPosition(),
                getHardware().bottomRightMotor.getCurrentPosition()
        };
    }

    public void setFieldCentric(boolean fc) {
        this.fieldCentric = fc;
    }

    public void setTargetPower(Vector3 targetPower){
        this.targetPower = targetPower;
    }

    /**
     * this method is a direct connection to the drive and bypasses things like motion smoothing. Use {@link Drive#setTargetPower(Vector3)}
     */
    public void moveRobot(double x, double y, double r){
        if (fieldCentric) {
            double[] rot = fieldCentricRotation(x, y);
            x = rot[0];
            y = rot[1];
        }
        double[] pows = dpc.convert(x,y,r);
        getHardware().topLeftMotor.setPower(pows[0]);
        getHardware().topRightMotor.setPower(pows[1]);
        getHardware().bottomLeftMotor.setPower(pows[2]);
        getHardware().bottomRightMotor.setPower(pows[3]);
    }

    public double[] fieldCentricRotation(double x, double y){
        // Get IMU angle in radians
        BNO055IMU imu = parent.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        double theta = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        // rotate coordinate system - IMU positive angle is CW
        double temp = x * Math.sin(theta) + y * Math.cos(theta);
        x = x * Math.cos(theta) - y * Math.sin(theta);
        y = temp;
        return (new double[]{x,y});
    }

    /**
     * just like the original moveRobot with different parameter type
     * @param powers the power that you want to move the robot at
     * @see Drive#moveRobot(double, double, double)
     */
    public void moveRobot(Vector3 powers){
        moveRobot(powers.X, powers.Y, powers.Z);
    }


    public void stopRobot(){
        xRamp.setCurrentVal(0);
        yRamp.setCurrentVal(0);
        rRamp.setCurrentVal(0);
        moveRobot(0,0,0);
    }

    @Override
    public void onSettingsUpdate(DriveSettings driveSettings) {
        //set smoothing values
        xRamp.setRamp(driveSettings.smoothingValues.X);
        yRamp.setRamp(driveSettings.smoothingValues.Y);
        rRamp.setRamp(driveSettings.smoothingValues.Z);

        //set smoothing
        if(driveSettings.useSmoothing){
            powerFilter = (pow) -> new Vector3(
                    xRamp.apply(pow.X),
                    yRamp.apply(pow.Y),
                    rRamp.apply(pow.Z)
            );
        } else {
            powerFilter = (pow) -> pow;
        }

        //set the conversion function
        switch (driveSettings.driveMode){
            case TANK:
                dpc = (x, y, r) -> (
                        new double[]{
                                x + r,
                                x - r,
                                x + r,
                                x - r
                        }
                );
                break;
            case MECANUM:
                dpc = (x, y, r) -> (
                        new double[]{
                                x + y + r,
                                -x + y - r,
                                -x + y + r,
                                x + y - r
                        }
                );
                break;
            case OMNI:
                //experimental
                dpc = (x, y, r) -> (
                        new double[]{
                                y + r,
                                x + r,
                                y + r,
                                x + r
                        }
                );
                break;
        }
    }

    @Override
    public void onHardwareUpdate(DriveHardware driveHardware) {

    }

    @Override
    public void onRun() {
        moveRobot(powerFilter.apply(targetPower));
    }

    @Override
    public void onRun(DriveControl control) {
        //parent.opMode.telemetry.addData("drive powers", control.power);//TODO remove

        if(control.stop) stopRobot();
        else setTargetPower(control.power);
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onStop() {

    }


    interface DrivePowerConverter{
        double[] convert(double x, double y, double r);
    }
}
