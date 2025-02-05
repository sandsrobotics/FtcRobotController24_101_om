package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;

import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

@Disabled
@Autonomous(name="27050 Specimen Humanside 2", group="27050")
public class ClawAutoSpec2 extends ClawAutoSpec {
    Vector3 observationzonepickup = new Vector3(45.5, -61, 90); // to stop wall hit

    @Override
    public void initAuto() {
        transformFunc = (v) -> v;
        bucketSample = false;
        fieldStartPos = observationzonepickup;
    }
}