package om.self.ezftc.utils.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public interface MotorFunction {
    Object run(List<DcMotor> motors, Object value, boolean separateValues);

    MotorFunction setMotorPower = (motors, value, separateValues) -> {
        for (int i = 0; i < motors.size(); i++)
            if (separateValues) {
                motors.get(i).setPower(((double[]) value)[i]);
            }else
                motors.get(i).setPower((double) value);
        return null;
    };

    MotorFunction setTargetPosition = (motors, value, separateValues) -> {
        for (int i = 0; i < motors.size(); i++)
            if (separateValues)
                motors.get(i).setTargetPosition(((int[]) value)[i]);
            else
                motors.get(i).setTargetPosition((int) value);
        return null;
    };

    MotorFunction getMotorPositions = (motors, value, separateValues) -> {
        int[] pos = new int[motors.size()];
        for (int i = 0; i < motors.size(); i++)
            pos[i] = motors.get(i).getCurrentPosition();
        return pos;
    };
}
