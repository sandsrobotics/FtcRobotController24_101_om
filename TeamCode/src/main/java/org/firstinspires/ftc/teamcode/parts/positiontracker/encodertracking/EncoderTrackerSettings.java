package org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking;

public class EncoderTrackerSettings {
    public final double ticksPerInchSideways;
    public final double ticksPerInchForward;

    public EncoderTrackerSettings(double ticksPerInchSideways, double ticksPerInchForward) {
        this.ticksPerInchSideways = ticksPerInchSideways;
        this.ticksPerInchForward = ticksPerInchForward;
    }

    public static EncoderTrackerSettings makeDefault(){
        return new EncoderTrackerSettings(
                52.38,
                45.68
        );
    }
}
