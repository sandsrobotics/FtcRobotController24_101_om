package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="1B  Claw Bucket", group="Test")
public class ClawBucketAuto2025 extends ClawAuto2025 {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
    }
}
