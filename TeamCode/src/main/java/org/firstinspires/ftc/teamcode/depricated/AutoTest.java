package org.firstinspires.ftc.teamcode.depricated;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import om.self.ezftc.utils.Vector3;

@Disabled
@Autonomous(name="6 TEST AUTO DONT RUN PLEASE!!!", group="Test")
public class AutoTest extends AutoRedWallAndAll {

    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        customStartPos = new Vector3(1.5, -1.5, 180);
        parkOnly = true;
        isBoard = false;
        isRed = false;
    }

}
