package org.firstinspires.ftc.teamcode.depricated;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import om.self.ezftc.utils.Vector3;

@Disabled
@Autonomous(name="2 RED-BOARD", group="Test")
public class AutoRedBoard extends AutoRedWallAndAll {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
        midPark = false;
        isRed = true;
        parkOnly = false;
        isBoard = true;
        extraPix = true;
        dropLow = true;
        stackPathSide = true;
        dropPathSide = true;
        stackSide = true;
        purple = false;
    }
}
