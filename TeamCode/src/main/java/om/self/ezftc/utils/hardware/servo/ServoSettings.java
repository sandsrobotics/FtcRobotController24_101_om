package om.self.ezftc.utils.hardware.servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import om.self.ezftc.utils.hardware.Valuable;

public class ServoSettings {
    public Number number;
    public Servo.Direction direction;
    public Double targetPos;

    public ServoSettings(Number number){
        construct(number, Servo.Direction.FORWARD, null);
    }

    public ServoSettings(Number number, Servo.Direction direction){
        construct(number,direction,null);
    }

    public ServoSettings(Number number, Servo.Direction direction, Double targetPos){
        construct(number,direction,targetPos);
    }

    public void construct(Number number, Servo.Direction direction, Double targetPos){
        this.number = number;
        this.direction = direction;
        this.targetPos = targetPos;

    }

    public Servo makeServo(HardwareMap hardwareMap){
        Servo servo = hardwareMap.get(Servo.class, number.value);
        updateServo(servo);
        return servo;
    }

    public ServoSSR makeServoSSR(HardwareMap hardwareMap){
        ServoSSR servo = new ServoSSR(hardwareMap.get(Servo.class, number.value));
        updateServo(servo);
        return servo;
    }

    public void updateServo(Servo servo){
        servo.setDirection(direction);
        if(targetPos != null)
            servo.setPosition(targetPos);
    }

    public enum Number implements Valuable {
        ZERO("servo0"),
        ONE("servo1"),
        TWO("servo2"),
        THREE("servo3"),
        FOUR("servo4"),
        FIVE("servo5"),
        ZERO_B("servo0B"),
        ONE_B("servo1B"),
        TWO_B("servo2B"),
        THREE_B("servo3B"),
        FOUR_B("servo4B"),
        FIVE_B("servo5B");

        public String value;

        public String getValue(){
            return value;
        }

        Number(String value){
            this.value = value;
        }
    }
}
