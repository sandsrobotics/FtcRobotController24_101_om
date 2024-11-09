package org.firstinspires.ftc.teamcode.parts.positionsolver;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.SolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;

public class XRelativeSolver extends Solver<Drive, DriveControl>{
    protected PositionTracker positionTracker;
    double start;

    public XRelativeSolver(Drive drive) {
        super(drive, "x relative channel");
        setSettings(new SolverSettings(0.6,10,true,10000, new PIDCoefficients(0.05, 0,0), 1));
    }

    public XRelativeSolver(Drive drive, SolverSettings settings){
        super(drive, "x relative channel");
        setSettings(settings);
    }

    @Override
    public void onBeanLoad() {
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    @Override
    public void onStart() {
        start = positionTracker.getRelativePosition().X;
    }

    @Override
    public double getError(double target) {
        double error = target - (positionTracker.getRelativePosition().X - start);
        parent.parent.opMode.telemetry.addData("relative x channel error", error);
        return error;
    }

    @Override
    public void move(DriveControl base) {
        base.power = base.power.addX(pid.returnValue());
    }

    @Override
    public ControllablePart<?, ?, ?, DriveControl> getControlled() {
        return parent;
    }

    @Override
    public void setNewTarget(double target, boolean resetPID) {
        super.setNewTarget(target, resetPID);
        triggerEvent(Robot.Events.START);
    }
}
