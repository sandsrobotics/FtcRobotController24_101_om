package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name="1B  Claw Bucket", group="Claw")
public class ClawAutoBucket extends ClawAutoSpec {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
    }
}
