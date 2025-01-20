package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="27050 Bucket", group="27050")
public class ClawAutoBucket extends ClawAutoSpec {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
    }
}
