package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import java.util.ArrayList;
import java.util.List;

public class MotorGroupEx extends MotorGroup{
    private List<Motor> motorList = new ArrayList<>();

    /**
     * Create a new MotorGroup with the provided Motors.
     *
     * @param leader    The leader motor.
     * @param followers The follower motors which follow the leader motor's protocols.
     */
    public MotorGroupEx(@NonNull Motor leader, Motor... followers) {
        super(leader, followers);
        motorList.add(leader);
        motorList.addAll(List.of(followers));
    }

    public int getCurrentPosition() {
        return motorList.get(0).getCurrentPosition();
    }

//    public void setRunMode(Motor.RunMode runmode) {
//        motorList.forEach(motor->motor.setRunMode(runmode));
//    }

//    public void set(double speed) {
//        motorList.get(0).set(speed);
//        double power = - motorList.get(0).get();
//        for (int i = 1; i < motorList.size(); i++) {
//            motorList.get(i).set(power);
//        }
//    }

}
