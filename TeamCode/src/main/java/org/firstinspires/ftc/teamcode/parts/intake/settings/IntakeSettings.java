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
    public final double flipperAutoSample        = flipperAlmostFloor;
    public final double flipperVertical          = 0.507; // 0.527
    public final double flipperBalanced          = 0.448; // 0.468
    public final double flipperParked            = 0.255; // 0.275
    public final int flipperSweepTime            = 1200; //1500;   // spec is 1250
    public final double flipperOffset            = 0.0;

    public final double chuteParked              = 0.677; //0.689;
    public final double chuteReady               = 0.535;
    public final double chuteDeposit             = 0.327;
    public final double chuteInspect             = 0.407;
    public final int chuteSweepTime              = 1000; //1500;   // spec is 1250
    public final double chuteOffset              = 0.0;

    public final double pinchFullOpen            = 0.364;
    public final double pinchReady               = 0.407;
    public final double pinchClosed              = 0.589;
    public final double pinchLoose               = 0.563;
    public final double pinchSuperLoose          = 0.545;
    public final int pinchSweepTime              = 1000; //1500;   // spec is 1250
    public final double pinchOffset              = 0.0;

    public final double parkDown                 = 0.665;
    public final double parkUp                   = 0.210;
    public final int parkSweepTime               = 400;
    public final double parkOffset               = 0.0;

    public final int positionSlideMin            = 10;
    public final int positionSlideMax            = 1200; // Physical limit = 1500; 1100 might be necessary if too long
    public final int positionSlideSpecimen       = 40;
    public final int positionSlideAutoSamplePickup = 900;
    public final int positionSlideAutoSampleRetract = 400;
    public final int positionSlideAutoSampleDeposit = 1100;
    public final int positionSlideStartIntake    = 450;   //todo: finalize number
    public final int positionSlidePitMin         = 250;    //todo: finalize number
    public final int toleranceSlide              = 20;
    public final int positionSlideOvershoot      = -10;
    public final int autoSampleSlideDistance     = 750; //todo : finalize number
    public final int autoSampleSlideMin          = positionSlideMin + 25;
    public final int positionSlideHome           = 20;

//    public final int positionLiftMin             = 10;
//    public final int positionLiftMax             = 2800; // 3000;  //4200; //4350;
//    public final int positionLiftReady           = 1500;
//    public final int positionLiftGetSpecimen     = 10;     //todo: finalize number
//    public final int positionLiftRaiseSpeciman   = 150; //= 50;
//    public final int positionLiftHangReady       = 1350; //1440;  //2500;   //todo: get number
//    public final int positionLiftHangRelease     = 980; //1000;  //2000;   //todo: get number
//    public final int positionLiftTransfer        = 10;
//    public final int positionLiftPreDump         = 2500; //2450;
//    public final int positionLiftHome            = 20;
//    public final int toleranceLift               = 20;
    public final int positionLiftMin             = (int)(10 * 435.0 / 1150);
    public final int positionLiftMax             = (int)(2800 * 435.0 / 1150); // 3000;  //4200; //4350;
    public final int positionLiftReady           = (int)(1500 * 435.0 / 1150);
    public final int positionLiftGetSpecimen     = (int)(10 * 435.0 / 1150);     //todo: finalize number
    public final int positionLiftRaiseSpecimen   = 100; //(int)(150 * 435.0 / 1150); //= 50;
    public final int positionLiftHangReady       = 550; //(int)(1350 * 435.0 / 1150); //1440;  //2500;   //todo: get number
    public final int positionLiftHangRelease     = (int)(980 * 435.0 / 1150); //1000;  //2000;   //todo: get number
    public final int positionLiftTransfer        = (int)(10 * 435.0 / 1150);
    public final int positionLiftPreDump         = (int)(2500 * 435.0 / 1150); //2450;
    public final int positionLiftHome            = (int)(20 * 435.0 / 1150);
    public final int toleranceLift               = (int)(20 * 435.0 / 1150);
    public final double liftSpecP_rue = 15;  // P for Run Using Encoder PIDF Spec
    public final double liftSpecP_rtp = 20;  // P for Run To Position PIDF Spec
    public final double liftSampleP_rue = 15;  // P for Run Using Encoder PIDF Sample todo: finalize number
    public final double liftSampleP_rtp = 16;  // P for Run To Position PIDF Sample todo: finalize number


    public final int positionHangMin             = 20;
    public final int positionHangMax             = 13500; //4350;
    public final int positionHangReady           = 10050; //9000; //todo: get number
    public final int positionHangFinal           = 2550; //745; //todo: get number
    public final int positionHangClear           = 12700;
    public final int toleranceHang               = 20;

    public final double distSampleGood           = 1.5;
    public final double distSampleUnload         = 2.0; //3.0;  // todo: get good number
    public final double distSampleEmpty          = 5.5;  // todo: verify it doesn't see tubing

    public IntakeSettings() {
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings();
    }
}

