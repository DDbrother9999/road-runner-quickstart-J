package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CombinedActionTeleOpMode;

import edu.nobles.robotics.ActionEx;


@Config
public class SlideMotor {

    //Feedforward
    public static double kS = 1; //static friction
    public static double kV = 0; //velocity
    public static double kA = 0; //acceleration

    private final String deviceName;
    private final Telemetry telemetry;
    public MotorGroupEx slideMotor;

    public SlideMotor(MotorGroupEx inSlideMotorGroup, Telemetry telemetry, String deviceName) {
        slideMotor = inSlideMotorGroup;
        this.deviceName = deviceName;

        this.telemetry = telemetry;
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.stopAndResetEncoder();

        slideMotor.set(0);
    }

    public void setManualMode() {
        if (slideMotor.runmode != Motor.RunMode.RawPower) {
            RobotLog.i(deviceName + " setManualMode ");
            slideMotor.setRunMode(Motor.RunMode.RawPower);
        }
    }

    public void setActionMode() {
        if (slideMotor.runmode != Motor.RunMode.PositionControl) {
            RobotLog.i(deviceName + " setActionMode ");
            slideMotor.setRunMode(Motor.RunMode.PositionControl);
            slideMotor.setPositionCoefficient(CombinedActionTeleOpMode.vertSlide_kP);
            slideMotor.setPositionTolerance(CombinedActionTeleOpMode.vertSlide_positionTolerance); // allowed maximum error
        }
    }

    public void zeroPowerWithFloat() {
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        slideMotor.set(0);
    }

    public String getDeviceName() {
        return deviceName;
    }

    //Universal MoveSlide Action (doesn't care about direction)

    public class PosMoveSlideAction implements ActionEx {
        int targetPosition;    //int for desired tick count
        double maxPower;
        double lastPowerSet = 0;
        boolean inited = false;

        public PosMoveSlideAction(int targetPosition, double maxPower) {
            this.targetPosition = targetPosition;
            this.maxPower = maxPower;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int current = slideMotor.getCurrentPosition();
            if (!inited) {
                RobotLog.i(deviceName + " Initial Position: " + current);
                RobotLog.i(deviceName + " To Position: " + targetPosition);
                telemetry.addData(deviceName + " To Position:", targetPosition);
                setActionMode();
                slideMotor.setTargetPosition(targetPosition);
                inited = true;
            }

            telemetry.addData(deviceName + " Current Position:", current);

            RobotLog.i(deviceName + " Current Position: " + current);

            if (slideMotor.atTargetPosition()) {
                slideMotor.stopMotor();

                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                telemetry.update();
                lastPowerSet = 0;
                return false;
            }

            slideMotor.set(maxPower);
            double actualPower = slideMotor.get();
            telemetry.addData(deviceName + " Power:", actualPower);
            RobotLog.i(deviceName + " Power: " + actualPower);

            telemetry.update();

            /*
            Basic cache to prevent repeats (not working, not sure why)

            if (lastPowerSet != maxPower) {
                slideMotor.set(maxPower);
                lastPowerSet=maxPower;
            }

             */

//            RobotLog.i(deviceName + " maxPower: " + maxPower);
//            double corrected = ((MotorGroup) slideMotor).iterator().next().getCorrectedVelocity();
//            RobotLog.i(deviceName + " corrected: " + corrected);
//            RobotLog.i(deviceName + " power: " + slideMotor.get());

            return true;
        }
    }


    public ActionEx moveSlide(int targetPosition, double maxPower) {
        return new PosMoveSlideAction(targetPosition, maxPower);
    }
}
