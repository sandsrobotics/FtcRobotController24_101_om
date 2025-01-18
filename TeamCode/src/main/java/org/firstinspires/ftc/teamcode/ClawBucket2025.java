package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="2b  Claw Bucket", group="Test")
public class ClawBucket2025 extends ClawAuto2025 {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
    }
}
