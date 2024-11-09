package org.firstinspires.ftc.teamcode.parts.dropper;

import org.firstinspires.ftc.teamcode.parts.dropper.hardware.DropperHardware;
import org.firstinspires.ftc.teamcode.parts.dropper.settings.DropperSettings;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.StatefullPart;
import om.self.task.core.Group;

public class Dropper extends StatefullPart<Robot, DropperSettings, DropperHardware, DropperState> {

    public Dropper(Robot parent) {
        super(parent, "Dropper");
    }

    public Dropper(Robot parent, Group taskManager){
        super(parent, "Dropper", taskManager);
    }





    @Override
    public void onBeanLoad() {

    }

    /**
     * WARNING: beans may not be loaded onInit, please use onStart for beans
     */
    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {

    }

    @Override
    public void onStop() {

    }

    @Override
    public DropperState getCurrentState() {
        return null;
    }

    @Override
    public void onStateUpdate(DropperState dropperState) {

    }

    @Override
    public DropperState sanitizeState(DropperState dropperState) {
        return null;
    }
}
