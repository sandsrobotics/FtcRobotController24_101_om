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
    public void onBeanLoad() {
    }

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
        // e-stop, either driver
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // *** DRIVER 2 CONTROLS ***
        // Driver 2 - slide control
        parent.setUserSlidePower(-parent.parent.opMode.gamepad2.left_stick_y);
        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            // Driver 2
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
            if (buttonMgr.getState(2, Buttons.right_stick_button, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.lowDumpIntake.restart();
            }
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                parent.setSpinner(parent.getSettings().spinnerOut);
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                parent.setSpinner(parent.getSettings().spinnerIn);
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasHeld)) {
                parent.setSpinner(parent.getSettings().spinnerOff);
            }
        }
        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {
            if (buttonMgr.getState(2, Buttons.dpad_up, State.wasSingleTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToHangRobotTask.restart();
            }
            if (buttonMgr.getState(2, Buttons.dpad_right, State.wasSingleTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.hangRobotTask.restart();
            }
            // this is for testing the autonomous sample task
            if (buttonMgr.getState(2, Buttons.dpad_down, State.wasDoubleTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.autonomousSample.restart();
            }
            // this is for inspection to show the maximum extent of the robot
            if (buttonMgr.getState(2, Buttons.dpad_left, State.wasDoubleTapped)) {
                parent.stopAllIntakeTasks();
                parent.getHardware().park.setPosition(parent.getSettings().parkUp);
                parent.getHardware().chute.setPosition(parent.getSettings().chuteDeposit);
                parent.setSlidePosition(parent.getSettings().positionSlideMax);
            }
            // emergency home
            if (buttonMgr.getState(2, Buttons.x, State.wasDoubleTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.startAutoHome();
            }
        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            // Driver 1
            if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToGetSpecimen.restart();
            }
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.getSpecimen.restart();
            }
            if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToHangSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.hangSpecimenTask.restart();
            }
        }
        // Driver 1 - start button is a "shift" key; anything below is when start is held first
        else {
            // add shifted controls here
        }
    }
}