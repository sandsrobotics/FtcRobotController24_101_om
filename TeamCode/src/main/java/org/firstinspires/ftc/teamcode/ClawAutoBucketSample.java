package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="27050 Bucket Sample", group="27050")
public class ClawAutoBucketSample extends ClawAutoBucket {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
        bucketSample = true;
    }
}
