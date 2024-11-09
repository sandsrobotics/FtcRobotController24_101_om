package om.self.ezftc.utils.hardware.servo;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public interface ServoFunction {
    Object run(List<Servo> servos, Object value, boolean separateValues);

    ServoFunction setServoPosition = (servos, value, separateValues) -> {
        for(int i = 0; i < servos.size(); i++)
            if(separateValues)
                servos.get(i).setPosition(((double[])value)[i]);
            else
                servos.get(i).setPosition((double)value);
        return null;
    };

    ServoFunction getServoPositions = (servos, value, separateValues) -> {
        double[] pos = new double[servos.size()];
        for (int i = 0; i < servos.size(); i++)
            pos[i] = servos.get(i).getPosition();
        return pos;
    };
}
