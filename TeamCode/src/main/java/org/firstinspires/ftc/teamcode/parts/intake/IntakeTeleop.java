package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
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
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed) ||
                buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        //in telelop, disable positiontracker if Driver1 uses controls
        if (FlipbotSettings.isTeleOp()) {
            if (parent.parent.opMode.gamepad1.right_trigger+
                    parent.parent.opMode.gamepad1.left_trigger+
                    parent.parent.opMode.gamepad1.left_stick_x+
                    parent.parent.opMode.gamepad1.left_stick_y+
                    parent.parent.opMode.gamepad1.right_stick_x+
                    parent.parent.opMode.gamepad1.right_stick_y != 0) {
                parent.positionSolver.stopSolver();
            }
        }

        // *** DRIVER 2 CONTROLS ***
        // Driver 2 - slide control
//        parent.setUserSlidePower(-parent.parent.opMode.gamepad2.left_stick_y);
        parent.setUserSlidePower(parent.parent.opMode.gamepad2.right_trigger-parent.parent.opMode.gamepad2.left_trigger);
        // Driver 2 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(2, Buttons.start, State.isPressed)) {
            // Driver 2
            if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.dockTask.restart();
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
                parent.tasks.lowDumpIntakeTask.restart();
            }
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                if (parent.getHardware().spinner.isSetPosition(parent.getSettings().spinnerOut)) {
                    parent.setSpinner(parent.getSettings().spinnerOff);
                } else {
                    parent.setSpinner(parent.getSettings().spinnerOut);
                }
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                if (parent.getHardware().spinner.isSetPosition(parent.getSettings().spinnerIn)) {
                    parent.setSpinner(parent.getSettings().spinnerOff);
                } else {
                    parent.setSpinner(parent.getSettings().spinnerIn);
                }
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasHeld)) {
                parent.setSpinner(parent.getSettings().spinnerOff);
            }
        }
        // Driver 2 - start button is a "shift" key; anything below is when start is held first
        else {
            if (buttonMgr.getState(2, Buttons.dpad_up, State.wasTapped)) {
                parent.getHardware().park.setPosition(parent.getSettings().parkUp);
            }
            if (buttonMgr.getState(2, Buttons.dpad_right, State.wasTapped)) {
                parent.getHardware().park.setPosition(parent.getSettings().parkDown);
            }
            // this is for testing the autonomous sample task
            if (buttonMgr.getState(2, Buttons.dpad_down, State.wasDoubleTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.autonomousSampleTask.restart();
            }
            // this is for inspection to show the maximum extent of the robot
            if (buttonMgr.getState(2, Buttons.dpad_left, State.wasDoubleTapped)) {
                parent.stopAllIntakeTasks();
//                parent.getHardware().park.setPosition(parent.getSettings().parkUp);
                parent.getHardware().chute.setPosition(parent.getSettings().chuteInspect);
                parent.setSlidePosition(parent.getSettings().positionSlideMax, 0.25);
                parent.getHardware().flipper.setPosition(parent.getSettings().flipperAlmostFloor);
            }
            // emergency home
            if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.startAutoHome();
            }
            if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
                FlipbotSettings.isYellowGood = !FlipbotSettings.isYellowGood;
            }
            // test out-take speed
            if (buttonMgr.getState(2, Buttons.left_bumper, State.wasTapped)) {
                parent.testSpinnerOut += parent.sIncrement;
                parent.testSpinnerOut = Math.max(0, Math.min(parent.testSpinnerOut, 0.5));
            }
            if (buttonMgr.getState(2, Buttons.right_bumper, State.wasTapped)) {
                parent.testSpinnerOut -= parent.sIncrement;
                parent.testSpinnerOut = Math.max(0, Math.min(parent.testSpinnerOut, 0.5));
            }
        }

        // *** DRIVER 1 CONTROLS ***
        // Driver 1 - start button is a "shift" key; anything below is if start is not pushed
        if (!buttonMgr.getState(1, Buttons.start, State.isPressed)) {
            // Driver 1

            if (buttonMgr.getState(1, Buttons.right_bumper, State.wasTapped)) {
                parent.tasks.moveToPickupSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
                parent.tasks.moveToHangSpecimenTask.restart();
                parent.tasks.prepareToHangSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.left_bumper, State.wasHeld)) {
                // combines the reset position and move to hang; must be held > 0.5 sec
                parent.pinpoint.setPosition(parent.p_atObsZone);
                parent.tasks.moveToHangSpecimenTask.restart();
                parent.tasks.prepareToHangSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.dpad_left, State.wasTapped)) {
                // Reset Position to p_13: p_atObservationZone - Vector3(33.5, -61.5, 90);
                parent.pinpoint.setPosition(parent.p_atObsZone);
            }

            if (buttonMgr.getState(1, Buttons.x, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToGetSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.getSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.prepareToHangSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
                parent.stopAllIntakeTasks();
                parent.tasks.hangSpecimenTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
//                parent.stopAllIntakeTasks();
                parent.tasks.prepareToHangRobotTask.restart();
            }
            if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
//                parent.stopAllIntakeTasks();
                parent.tasks.hangRobotTask.restart();
            }
        }
        // Driver 1 - start button is a "shift" key; anything below is when start is held first
        else {
            // add shifted controls here
            if (buttonMgr.getState(1, Buttons.right_bumper, State.isPressed)) {
                parent.getRangeDistance();
            }
            if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
                // p up for run using encoders
                parent.pidf_rue = parent.getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
                parent.pidf_rue = new PIDFCoefficients(parent.pidf_rue.p + parent.pIncrement,
                        parent.pidf_rue.i, parent.pidf_rue.d, parent.pidf_rue.f, MotorControlAlgorithm.LegacyPID);
                parent.getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, parent.pidf_rue);
            }
            if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
                // p down for run using encoders
                parent.pidf_rue = parent.getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
                parent.pidf_rue = new PIDFCoefficients(parent.pidf_rue.p - parent.pIncrement,
                        parent.pidf_rue.i, parent.pidf_rue.d, parent.pidf_rue.f, MotorControlAlgorithm.LegacyPID);
                parent.getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, parent.pidf_rue);
            }
            if (buttonMgr.getState(1, Buttons.y, State.wasTapped)) {
                // p up for run to position
                parent.pidf_rtp = parent.getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
                parent.pidf_rtp = new PIDFCoefficients(parent.pidf_rtp.p + parent.pIncrement,
                        parent.pidf_rtp.i, parent.pidf_rtp.d, parent.pidf_rtp.f, MotorControlAlgorithm.LegacyPID);
                parent.getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, parent.pidf_rtp);
            }
            if (buttonMgr.getState(1, Buttons.a, State.wasTapped)) {
                // p down for run to position
                parent.pidf_rtp = parent.getHardware().liftMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
                parent.pidf_rtp = new PIDFCoefficients(parent.pidf_rtp.p -parent.pIncrement,
                        parent.pidf_rtp.i, parent.pidf_rtp.d, parent.pidf_rtp.f, MotorControlAlgorithm.LegacyPID);
                parent.getHardware().liftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, parent.pidf_rtp);
            }

        }
    }
}