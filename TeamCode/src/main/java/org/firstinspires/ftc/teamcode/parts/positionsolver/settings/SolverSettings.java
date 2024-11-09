package org.firstinspires.ftc.teamcode.parts.positionsolver.settings;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class SolverSettings {
    public final double tolerance;
    public final int reqTimesInTolerance;
    public final boolean alwaysRun;
    public final int maxRuntime;
    public final PIDCoefficients PIDCoefficients;
    public final double maxSpeed;

    public SolverSettings(double tolerance, int reqTimesInTolerance, boolean alwaysRun, int maxRuntime, com.qualcomm.robotcore.hardware.PIDCoefficients PIDCoefficients, double maxSpeed) {
        this.tolerance = tolerance;
        this.reqTimesInTolerance = reqTimesInTolerance;
        this.alwaysRun = alwaysRun;
        this.maxRuntime = maxRuntime;
        this.PIDCoefficients = PIDCoefficients;
        this.maxSpeed = maxSpeed;
    }
}
