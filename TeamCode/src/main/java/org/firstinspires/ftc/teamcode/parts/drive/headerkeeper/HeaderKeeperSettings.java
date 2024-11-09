package org.firstinspires.ftc.teamcode.parts.drive.headerkeeper;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class HeaderKeeperSettings {
    public final double minRegisterVal;

    public final PIDCoefficients pidCoefficients;
    public final double maxI;

    public final int headingSettleDelay;

    public HeaderKeeperSettings(double minRegisterVal, PIDCoefficients pidCoefficients, double maxI, int headingSettleDelay) {
        this.minRegisterVal = minRegisterVal;
        this.pidCoefficients = pidCoefficients;
        this.maxI = maxI;
        this.headingSettleDelay = headingSettleDelay;
    }

    public static HeaderKeeperSettings makeDefault(){
        return new HeaderKeeperSettings(
                0.1,
                new PIDCoefficients(
                        -0.02,0,0
                ),
                0.3,
                700
        );
    }
}
