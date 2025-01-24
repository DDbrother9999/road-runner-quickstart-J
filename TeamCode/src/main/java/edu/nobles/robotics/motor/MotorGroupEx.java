package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import java.util.ArrayList;
import java.util.List;

public class MotorGroupEx extends MotorGroup {
    private final List<MotorEx> motorList = new ArrayList<>();

    public RunMode runmode;

    /**
     * Create a new MotorGroup with the provided Motors.
     *
     * @param leader    The leader motor.
     * @param followers The follower motors which follow the leader motor's protocols.
     */
    public MotorGroupEx(@NonNull MotorEx leader, MotorEx... followers) {
        super(leader, followers);
        motorList.add(leader);
        motorList.addAll(List.of(followers));
    }

    public int getCurrentPosition() {
        return motorList.get(0).getCurrentPosition();
    }

    @Override
    public void setRunMode(RunMode runmode) {
        this.runmode = runmode;
        super.setRunMode(runmode);
    }

    @Override
    public void set(double speed) {
        if (runmode == RunMode.RawPower) {
            motorList.forEach(m -> m.set(speed));
        } else {
            super.set(speed);
        }
    }
}
