package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import java.util.ArrayList;
import java.util.List;

public class MotorGroupEx extends MotorGroup {
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

    @Override
    public int getCurrentPosition() {
        return motorList.get(0).getCurrentPosition();
    }

    @Override
    public void setRunMode(RunMode runmode) {
        motorList.forEach(motor->motor.setRunMode(runmode));
    }

    @Override
    public void set(double speed) {
        motorList.forEach(motor->motor.set(speed));
    }

}
