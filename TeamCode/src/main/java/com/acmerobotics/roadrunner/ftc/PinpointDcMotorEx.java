package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.jetbrains.annotations.NotNull;

public class PinpointDcMotorEx extends DcMotorImplEx {
    private final GoBildaPinpointDriverRR pinpoint;

    private final boolean usePerpendicular;

    public PinpointDcMotorEx(GoBildaPinpointDriverRR pinpoint, boolean usePerpendicular, Direction direction, DcMotorController dummyController) {
        super(dummyController, 0, direction);
        this.pinpoint = pinpoint;
        this.usePerpendicular = usePerpendicular;
        this.direction = direction;
    }

    @Override
    public synchronized int getCurrentPosition() {
        this.pinpoint.update();
        Pose2d absPose = this.pinpoint.getPositionRR();
        Vector2d rotatedPosVector = absPose.position;
        int pos = 0;
        if (this.usePerpendicular) {
            pos = (int)rotatedPosVector.y; // QUESTION: double
        } else {
            pos = (int)rotatedPosVector.x; // QUESTION: double
        }
        return pos;
    }

    @Override
    public synchronized double getVelocity() {
        this.pinpoint.update();
        PoseVelocity2d absVel = this.pinpoint.getVelocityRR();
        Vector2d rotatedVelVector = absVel.linearVel;
        double vel = 0.0;
        if (this.usePerpendicular) {
            vel = (int)rotatedVelVector.y;
        } else {
            vel = (int)rotatedVelVector.x;
        }
        return vel;
    }
}
