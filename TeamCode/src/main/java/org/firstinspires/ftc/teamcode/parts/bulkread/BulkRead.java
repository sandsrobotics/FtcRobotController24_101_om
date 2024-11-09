package org.firstinspires.ftc.teamcode.parts.bulkread;

import com.qualcomm.hardware.lynx.LynxModule;

import org.apache.commons.lang3.ObjectUtils;

import java.util.List;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class BulkRead extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    private List<LynxModule> allHubs;

    public BulkRead(Robot parent) {
        super(parent, "bulk read", parent.endTaskManager);
    }

    @Override
    public void onRun() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {
        allHubs = parent.opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onStop() {

    }
}
