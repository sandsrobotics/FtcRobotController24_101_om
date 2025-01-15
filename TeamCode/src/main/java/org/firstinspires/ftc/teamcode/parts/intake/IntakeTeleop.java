package org.firstinspires.ftc.teamcode.parts.intake;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
    int myDelay = 1000;
    ButtonMgr buttonMgr;
    public IntakeTeleop(Intake parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop(Intake parent, IntakeTeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl(
                (int) settings.sweepSpeedSupplier.get(),
                (int) settings.sweepLiftSupplier.get(),
                (int) settings.sweepSlideSupplier.get(),
                (int) settings.bucketLiftSupplier.get(),
     //           (int) settings.intakeSupplier.get(),
                (int) settings.pinchSupplier.get(),
                (int) settings.v_SlideSupplier.get(),
                (int) settings.intakeAngleSupplier.get()
        ), true);
    }

    @Override
    public void onRun() {
        driverControls();
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }

    public void driverControls() {
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }
        if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.safeTask.restart();
        }
        if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.prepareToIntakeTask.restart();
        }
        if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.autoIntakeTask.restart();
        }
        if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.transferTask.restart();
        }
        if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.prepareToDepositTask.restart();
        }
        if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
            parent.stopAllIntakeTasks();
            parent.tasks.depositTask.restart();
        }
    }
}
