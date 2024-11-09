package org.firstinspires.ftc.teamcode.depricated;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="3 BLUE-WALL", group="Test")
public class AutoBlueWall extends AutoRedWallAndAll {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v.withY(-v.Y).withZ(-v.Z);
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
        extraPix = true;
        dropLow = false;
        stackPathSide = false;
        dropPathSide = false;
        extraWallPix = false;
        stackSide = false;
        purple = false;
    }

}
