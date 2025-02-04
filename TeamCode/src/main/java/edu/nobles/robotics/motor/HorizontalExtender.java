package edu.nobles.robotics.motor;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;

import java.util.function.Supplier;

import edu.nobles.robotics.ActionEx;

public class HorizontalExtender {
    private final String deviceName;
    private final Telemetry telemetry;
    public MotorGroupEx extender;

    public boolean available = true;

    public HorizontalExtender(MotorGroupEx inSlideMotorGroup, Telemetry telemetry, String deviceName) {
        try {
            extender = inSlideMotorGroup;
            extender.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            extender.stopAndResetEncoder();
            this.deviceName = "Unavailable HorizontalExtender";
            this.telemetry=telemetry;
            extender.set(0);
        } catch (Exception e) {
            available = false;
            RobotLog.ee(RobotLog.TAG, e, "HorizontalExtender can't start");
        }
    }

    public void setPower(double power) {
        if (Math.abs(power) < 0.01) {
            extender.set(0);
        } else if (power > 0) {
            extender.set(power);
        } else {
            extender.set(power * ANewActionTeleOpMode.intakeSlide_retractFactor);
        }
    }

    public ActionEx runForTime(long runForTime, double power) {
        return new HorizontalExtenderRunForTimeAction(runForTime, power);
    }

    public ActionEx runWithJoystick(Supplier<Float> joystickPosition, double maxPower) {
        return new HorizontalExtenderWithJoystickAction(joystickPosition, maxPower);
    }

    public class HorizontalExtenderRunForTimeAction implements ActionEx {
        long runForTime;
        double power;

        boolean initialized;
        long endTime;

        /**
         * @param runForTime in millisecond
         * @param power      constant power
         */
        public HorizontalExtenderRunForTimeAction(long runForTime, double power) {
            this.runForTime = runForTime;
            this.power = power;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!available)
                return false;

            if (!initialized) {
                endTime = System.currentTimeMillis() + runForTime;
                setPower(power);
                initialized = true;
            }

            if (System.currentTimeMillis() > endTime)
                return false;

            return true;
        }
    }

    public class HorizontalExtenderWithJoystickAction implements ActionEx {
        Supplier<Float> joystickPosition;
        double maxPower;

        public HorizontalExtenderWithJoystickAction(Supplier<Float> joystickPosition, double maxPower) {
            this.joystickPosition = joystickPosition;
            this.maxPower = maxPower;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!available)
                return false;

            setPower(maxPower * joystickPosition.get());

            return true;
        }
    }


}
