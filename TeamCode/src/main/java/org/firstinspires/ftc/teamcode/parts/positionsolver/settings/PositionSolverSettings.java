
package org.firstinspires.ftc.teamcode.parts.positionsolver.settings;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PositionSolverSettings {
    public final SolverSettings xChannelSettings;
    public final SolverSettings yChannelSettings;
    public final SolverSettings rChannelSettings;

    public PositionSolverSettings(SolverSettings xChannelSettings, SolverSettings yChannelSettings, SolverSettings rChannelSettings) {
        this.xChannelSettings = xChannelSettings;
        this.yChannelSettings = yChannelSettings;
        this.rChannelSettings = rChannelSettings;
    }

    public static PositionSolverSettings loseSettings = new PositionSolverSettings(
            new SolverSettings(5, 5, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(5, 5, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 5, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings extraLoseSettings = new PositionSolverSettings(
            new SolverSettings(10, 1, true, 10000, new PIDCoefficients(0.5, 0, 0), 1),
            new SolverSettings(10, 1, true, 10000, new PIDCoefficients(0.5, 0, 0), 1),
            new SolverSettings(2.5, 1, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings slowSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .7),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .7),
            new SolverSettings(2, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), .7)
    );

    public static PositionSolverSettings superSlowSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(2, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), .5)
    );

    public static PositionSolverSettings defaultSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings defaultTwiceSettings = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings defaultTwiceSlowSettings = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), .5)
    );

    public static PositionSolverSettings defaultTwiceNoAlwaysRunSettings = new PositionSolverSettings(
            new SolverSettings(1, 2, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 2, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings defaultNoAlwaysRunSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 10, false, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings slowNoAlwaysRunSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), .4),
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), .4),
            new SolverSettings(2.5, 10, false, 10000, new PIDCoefficients(0.0125, 0, 0), .4)
    );

    public static PositionSolverSettings slowTestSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .5),
            new SolverSettings(2.5, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), .5)
    );

    public static PositionSolverSettings defaultTwiceSettingsHang = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1.0)
    );

    public static PositionSolverSettings defaultTwiceSettingsHang1 = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.035, 0, 0), 0.8),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.035, 0, 0), 0.8),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 0.8)
    );

    public static PositionSolverSettings lkSloppyY = new PositionSolverSettings(
            new SolverSettings(2, 1, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(4, 1, true, 10000, new PIDCoefficients(0.1, 0, 0), 1.0),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1.0)
    );

    public static PositionSolverSettings lkSloppyX = new PositionSolverSettings(
            new SolverSettings(4, 1, true, 10000, new PIDCoefficients(0.1, 0, 0), 1.0),
            new SolverSettings(2, 1, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1.0)
    );

    public static PositionSolverSettings lkSloppyZ = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0),
            new SolverSettings(5, 1, true, 10000, new PIDCoefficients(0.05, 0, 0), 1.0)
    );

    public static PositionSolverSettings lkD2MoveBias = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 0.5)
    );

    public static PositionSolverSettings lkD2RotateBias = new PositionSolverSettings(
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 0.5),
            new SolverSettings(1, 2, true, 10000, new PIDCoefficients(0.05, 0, 0), 0.5),
            new SolverSettings(2.5, 2, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );


}
