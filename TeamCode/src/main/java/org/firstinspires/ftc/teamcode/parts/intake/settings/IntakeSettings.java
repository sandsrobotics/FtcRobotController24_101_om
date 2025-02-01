package org.firstinspires.ftc.teamcode.parts.intake.settings;

public class  IntakeSettings {

    public final double spinnerIn                = 1;
    public final double spinnerOff               = 0.5;
    public final double spinnerOut               = 0;
    public final double spinnerSlowOut           = 0.35;  //todo: finalize number
    public final int spinnerSweepTime            = 100;  //probably not relevant

    public final double flipperFloor             = 0.839;  // 0.859
    public final double flipperAlmostFloor       = 0.824; // 0.844
    public final double flipperSafe              = 0.678; // 0.698
    public final double flipperVertical          = 0.507; // 0.527
    public final double flipperBalanced          = 0.448; // 0.468
    public final double flipperParked            = 0.255; // 0.275
    public final int flipperSweepTime            = 1500;   // spec is 1250
    public final double flipperOffset            = 0.0;

    public final double chuteParked              = 0.689;
    public final double chuteReady               = 0.535;
    public final double chuteDeposit             = 0.327;
    public final double chuteInspect             = 0.407;
    public final int chuteSweepTime              = 1500;   // spec is 1250
    public final double chuteOffset              = 0.0;

    public final double pinchFullOpen            = 0.364;
    public final double pinchReady               = 0.407;
    public final double pinchClosed              = 0.589;
    public final double pinchLoose               = 0.563;
    public final double pinchSuperLoose          = 0.545;
    public final int pinchSweepTime              = 1500;   // spec is 1250
    public final double pinchOffset              = 0.0;

    public final double parkDown                 = 0.208;
    public final double parkUp                   = 0.756;
    public final int parkSweepTime               = 400;
    public final double parkOffset               = 0.0;

    public final int positionSlideMin            = 10;
    public final int positionSlideMax            = 1200; // Physical limit = 1500; 1100 might be necessary if too long
    public final int positionSlideSpecimen       = 40;
    public final int positionSlideStartIntake    = 450;   //todo: finalize number
    public final int positionSlidePitMin         = 250;    //todo: finalize number
    public final int toleranceSlide              = 20;
    public final int positionSlideOvershoot      = -10;
    public final int autoSampleSlideDistance     = 750; //todo : finalize number

    public final int positionLiftMin             = 10;
    public final int positionLiftMax             = 2800; // 3000;  //4200; //4350;
    public final int positionLiftReady           = 1500;
    public final int positionLiftGetSpecimen     = 10;     //todo: finalize number
    public final int positionLiftRaiseSpeciman   = 150; //= 50;
    public final int positionLiftHangReady       = 1440;  //2500;   //todo: get number
    public final int positionLiftHangRelease     = 980; //1000;  //2000;   //todo: get number
    public final int positionLiftTransfer        = 10;
    public final int positionLiftPreDump         = 2450;
    public final int toleranceLift               = 20;

    public final int positionHangMin             = 20;
    public final int positionHangMax             = 13500; //4350;
    public final int positionHangReady           = 10050; //9000; //todo: get number
    public final int positionHangFinal           = 2550; //745; //todo: get number
    public final int positionHangClear           = 12700;
    public final int toleranceHang               = 20;

    public final double distSampleGood           = 1.5;
    public final double distSampleUnload         = 3.0;  // todo: get good number
    public final double distSampleEmpty          = 5.5;  // todo: verify it doesn't see tubing

    public IntakeSettings() {
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings();
    }
}

