package com.acmerobotics.roadrunner.ftc;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import edu.nobles.robotics.TuningParameter;

public class PinpointDcMotorEx extends DcMotorImplEx {
    private final GoBildaPinpointDriverRR pinpoint;

    private final boolean usePerpendicular;

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

        if (this.usePerpendicular) {
            return pinpoint.getVelY() * TuningParameter.current.pinpointParams.encoderResolution;
        } else {
            return pinpoint.getVelX() * TuningParameter.current.pinpointParams.encoderResolution;
        }

    }
}
