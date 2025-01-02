package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public class SlideMotor {

    public static double target;
    public static double kP = 0.05;
    public static double positionTolerance = 15;   // allowed maximum error
    public static int fullExtend = 1000; //distance to full extension

    private final String deviceName;
    private final MotorEx slideMotor;
    private final Telemetry telemetry;
    private final MotorEx.Encoder encoder;

    public SlideMotor(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, boolean isInverted) {
        this.deviceName = deviceName;
        slideMotor = new MotorEx(hardwareMap, "motorOne");

        this.telemetry = telemetry;
        slideMotor.setInverted(isInverted);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.resetEncoder();
        encoder = slideMotor.encoder;
        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setPositionCoefficient(kP);
        slideMotor.set(0);
        slideMotor.setPositionTolerance(positionTolerance); // allowed maximum error
    }

    public boolean getInverted(){
        return slideMotor.getInverted();
    }

    public String getDeviceName() {
        return deviceName;
    }

    /**
     * Unit is Degree
     */
    public double getPosition() {
        return slideMotor.getCurrentPosition();
    }

    //Universal MoveSlide Action (doesn't care about direction)

    public class MoveSlide implements Action {
        int targetPosition;    //int for desired tick count
        long nextActionTime;
        double maxPower;

        double lastPowerSet = 0;

        int direction;

        public MoveSlide(int targetPosition, double maxPower) {
            this.targetPosition = targetPosition;
            this.maxPower = maxPower;

        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double current = slideMotor.getCurrentPosition();

            telemetry.addData(deviceName + " Last Power Set:", lastPowerSet);
            telemetry.addData(deviceName + " Current Position:", current);
            telemetry.addData(deviceName + " To Position:", targetPosition);

            RobotLog.i(deviceName + " Current Position: %.1f, To: %.1f", current, targetPosition);

            if (slideMotor.atTargetPosition()) {
                slideMotor.stopMotor();

                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                telemetry.update();
                return false;
            }
            telemetry.update();

            //adjusts for direction
            double motorPower;
            if(targetPosition>current){
                motorPower = maxPower;
            }
            else{
                motorPower=-maxPower;
            }

            //Basic cache to prevent repeats
            if (lastPowerSet != motorPower) {
                slideMotor.set(motorPower);
                lastPowerSet=motorPower;
            }

            return true;
        }
    }

    public Action MoveSlide(int targetPosition, double maxPower) {
        return new edu.nobles.robotics.motor.SlideMotor.MoveSlide(targetPosition, maxPower);
    }
}
