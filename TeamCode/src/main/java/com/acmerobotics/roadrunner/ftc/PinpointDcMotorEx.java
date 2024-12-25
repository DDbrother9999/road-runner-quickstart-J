package com.acmerobotics.roadrunner.ftc;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import edu.nobles.robotics.TuningParameter;

public class PinpointDcMotorEx extends DcMotorImplEx {
    private final GoBildaPinpointDriverRR pinpoint;

    private final boolean usePerpendicular;

    private double prevVel = 0.0;

    public PinpointDcMotorEx(GoBildaPinpointDriverRR pinpoint, boolean usePerpendicular, DcMotorController dummyController) {
        super(dummyController, 0, Direction.FORWARD);
        this.pinpoint = pinpoint;
        this.usePerpendicular = usePerpendicular;
    }

    @Override
    public synchronized int getCurrentPosition() {
        this.pinpoint.update();

        if (this.usePerpendicular) {
            return pinpoint.getEncoderY();
        } else {
            return pinpoint.getEncoderX();
        }
        /*
        if (this.usePerpendicular) {
            return (int)(pinpoint.getPosY() * TuningParameter.current.pinpointParams.encoderResolution);
        } else {
            return (int)(pinpoint.getPosX() * TuningParameter.current.pinpointParams.encoderResolution);
        }
        */
    }

    @Override
    public synchronized double getVelocity() {
        // pinpoint.update();
        double vel;
        if (this.usePerpendicular) {
            vel = adjustVelocity(pinpoint.getVelY());
        } else {
            vel = adjustVelocity(pinpoint.getVelX());
        }
        prevVel = vel;
        return vel;
    }

    private double adjustVelocity(double origVel) {
        double vel = origVel * TuningParameter.current.pinpointParams.encoderResolution;

        // Road Runner's Tuning code does some strange manipulation on velocity in following code. It assumes velocity number is always a multiple of 20
        // due to Expansion Hub's 50ms measurement window. However Pinpoint device has faster measurement window. This causes several tuning OpModes, such
        // as ForwardRampLogger, LateralRampLogger, show wrong velocity. We round velocity by 20 here to solve this issue.
        // See https://github.com/acmerobotics/road-runner-ftc/blob/c9f0be75158276c5dfcd82ccabb639a15a200f98/web/tuning/common.ts#L65C1-L74C2
        vel = ((int) (vel / 20)) * 20.0;

        if (Math.abs(prevVel - vel) > 0.1) {
            String type = this.usePerpendicular ? "Perp" : "Par";
            RobotLog.i(type + " velocity:" + vel);
        }
        return vel;
    }


}
