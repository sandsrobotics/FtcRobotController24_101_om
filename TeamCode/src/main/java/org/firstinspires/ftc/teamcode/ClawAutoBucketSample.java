package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import om.self.ezftc.utils.Vector3;

@Disabled
@Autonomous(name="27050 Bucket Sample", group="27050")
public class ClawAutoBucketSample extends ClawAutoBucket {
    Vector3 bucketsamplestart = new Vector3(-37.5, -62, 90);
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        bucketSide = true;
        bucketSample = true;
        fieldStartPos = bucketsamplestart;
    }
}
