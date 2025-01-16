package edu.nobles.robotics.servo;
/*
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.nobles.robotics.motor.SlideMotor;

public class ContServo {
    private CRServo crservo;
    private String deviceName;
    private Telemetry telemetry;

    public ContServo(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, boolean inverted) {
        this.deviceName = deviceName;
        crservo = new CRServo(hardwareMap, deviceName);
        this.telemetry = telemetry;
        crservo.setInverted(inverted);
    }

    public boolean getInverted() {
        return crservo.getInverted();
    }

    public double getPower(){
        return crservo.get();
    }

    public class PosMoveContServoAction implements Action {
        int targetPosition;    //int for desired tick count
        double maxPower;

        double lastPowerSet = 0;
        public PosMoveContServoAction(int targetPosition, double maxPower) {
            this.targetPosition = targetPosition;
            this.maxPower = maxPower;

        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            telemetry.addData(deviceName + " Last Power Set:", lastPowerSet);
            telemetry.addData(deviceName + " Current Position:", current);
            telemetry.addData(deviceName + " To Position:", targetPosition);

            RobotLog.i(deviceName + " Last Power Set " + lastPowerSet);
            RobotLog.i(deviceName + " To Position: " + targetPosition);

            if (slideMotor.atTargetPosition()) {
                slideMotor.stopMotor();

                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                telemetry.update();
                lastPowerSet = 0;
                return false;
            }
            telemetry.update();


            slideMotor.set(maxPower);


import com.arcrobotics.ftclib.hardware.motors.MotorGroup;RobotLog.i(deviceName + " maxPower: " + maxPower);
            double corrected = ((MotorGroup)slideMotor).iterator().next().getCorrectedVelocity();
            RobotLog.i(deviceName + " corrected: " + corrected);
            RobotLog.i(deviceName + " power: " + slideMotor.get());

            return true;
        }
    }


    public Action PosMoveContServo(int targetPosition, double maxPower) {
        return new SlideMotor.PosMoveContServoAction(targetPosition, maxPower);
    }
}

}

*/