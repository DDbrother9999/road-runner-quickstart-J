package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public class SlideMotor {

    //Position Controller
    public static double kP = 0.05;

    //Feedforward
    public static double kS = 1; //static friction
    public static double kV = 0; //velocity
    public static double kA = 0; //acceleration

    public static double positionTolerance = 15;   // allowed maximum error

    public static int extendLimit = 1000; //position at full extension
    public static int retractLimit = 0; //position at full retraction


    private final String deviceName;
    private final MotorEx slideMotor;
    private final Telemetry telemetry;
    private  MotorEx.Encoder encoder;

    public SlideMotor(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, boolean isInverted) {
        this.deviceName = deviceName;
        slideMotor = new MotorEx(hardwareMap, deviceName);

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

    public class PosMoveSlideAction implements Action {
        int targetPosition;    //int for desired tick count
        double maxPower;

        double lastPowerSet = 0;
        public PosMoveSlideAction(int targetPosition, double maxPower) {
            this.targetPosition = targetPosition;
            this.maxPower = maxPower;

            slideMotor.setTargetPosition(targetPosition);
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            int current = slideMotor.getCurrentPosition();

            telemetry.addData(deviceName + " Last Power Set:", lastPowerSet);
            telemetry.addData(deviceName + " Current Position:", current);
            telemetry.addData(deviceName + " To Position:", targetPosition);

            RobotLog.i(deviceName + " Last Power Set " + lastPowerSet);
            RobotLog.i(deviceName + " Current Position: " + current);
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

            /*
            Basic cache to prevent repeats

            if (lastPowerSet != maxPower) {
                slideMotor.set(maxPower);
                lastPowerSet=maxPower;
            }

             */

            RobotLog.i(deviceName + " maxPower: " + maxPower);
            double corrected = slideMotor.getCorrectedVelocity();
            RobotLog.i(deviceName + " corrected: " + corrected);



            return true;
        }
    }


    public Action PosMoveSlide(int targetPosition, double maxPower) {
        return new edu.nobles.robotics.motor.SlideMotor.PosMoveSlideAction(targetPosition, maxPower);
    }
}
