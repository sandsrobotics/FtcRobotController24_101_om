package org.firstinspires.ftc.teamcode.parts.intake;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleopDemo extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
    ButtonMgr buttonMgr;
    private boolean enableDemoDriver = false;

    public IntakeTeleopDemo(Intake parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings.makeDefault(parent.parent));
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleopDemo(Intake parent, IntakeTeleopSettings settings) {
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
                0,0,0,0,0,0,0
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
        if (buttonMgr.getState(1, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // Driver 1 will be the Team guide, Driver 2 will be the guest demo driver.
        // Driver 1 is set to override many things here and in drive control

        // Driver 1 right_bumper is a dead man switch for Driver 2
        if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
            enableDemoDriver = true;
            FlipbotSettings.demoDriverMultiplier = FlipbotSettings.demoDriverDefaultMultiplier;
        } else {
            enableDemoDriver = false;
            FlipbotSettings.demoDriverMultiplier = 0;
        }

        // Slide control
        if ((parent.parent.opMode.gamepad1.right_trigger-parent.parent.opMode.gamepad1.left_trigger) != 0) {
            parent.setUserSlidePower(parent.parent.opMode.gamepad1.right_trigger-parent.parent.opMode.gamepad1.left_trigger);
        } else if (enableDemoDriver) {
            parent.setUserSlidePower(parent.parent.opMode.gamepad2.right_trigger-parent.parent.opMode.gamepad2.left_trigger);
        } else {
            parent.setUserSlidePower(0);
        }

        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            // Start not pressed
            if (eitherGuestOrTeam(Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.dockTask.restart();
            }
            if (eitherGuestOrTeam(Buttons.y, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToIntakeTask.restart();
            }
            if (eitherGuestOrTeam(Buttons.b, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.autoIntakeTask.restart();
            }
            if (eitherGuestOrTeam(Buttons.a, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.transferTask.restart();
            }
            if (eitherGuestOrTeam(Buttons.dpad_up, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToDepositTask.restart();
            }
            if (eitherGuestOrTeam(Buttons.dpad_right, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.depositTask.restart();
            }
//            if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
//                if (parent.getHardware().spinner.isSetPosition(parent.getSettings().spinnerOut)) {
//                    parent.setSpinner(parent.getSettings().spinnerOff);
//                } else {
//                    parent.setSpinner(parent.getSettings().spinnerOut);
//                }
//            }
//            if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
//                if (parent.getHardware().spinner.isSetPosition(parent.getSettings().spinnerIn)) {
//                    parent.setSpinner(parent.getSettings().spinnerOff);
//                } else {
//                    parent.setSpinner(parent.getSettings().spinnerIn);
//                }
//            }
        }
        else {
            //start pressed
            // emergency home
            if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.startAutoHome();
            }
        }
    }

    public boolean eitherGuestOrTeam(Buttons button, State state) {
        // if either was enabled and activated the control, return true
        boolean team = buttonMgr.getState(1, button, state);
        boolean guest = enableDemoDriver && buttonMgr.getState(2, button, state);
        return team || guest;
    }

}