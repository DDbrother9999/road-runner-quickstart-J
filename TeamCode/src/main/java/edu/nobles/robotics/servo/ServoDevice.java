package edu.nobles.robotics.servo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Supplier;

import edu.nobles.robotics.ActionEx;

@Config
public class ServoDevice {
    private final String deviceName;
    private final Telemetry telemetry;
    public SimpleServo servo;

    public boolean available = true;

    private double practicalMinAngle;
    private double practicalMaxAngle;

    public ServoDevice(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        this(deviceName, hardwareMap, telemetry, 300); //All gobilda servos are 0 to 300
    }

    public ServoDevice(String deviceName, HardwareMap hardwareMap, Telemetry telemetry, double maxAngle) {
        this.deviceName = deviceName;
        this.telemetry = telemetry;
        this.practicalMinAngle = 0;
        this.practicalMaxAngle = maxAngle;

        try {
            servo = new SimpleServo(hardwareMap, deviceName, 0, maxAngle, AngleUnit.DEGREES); //All gobilda servos are 0 to 300
        } catch (Exception e) {
            available = false;
            RobotLog.e("Servo " + deviceName + " is not available");
            return;
        }
    }

    public void setPracticalAngles(double practicalMinAngle, double practicalMaxAngle) {
        this.practicalMinAngle = practicalMinAngle;
        this.practicalMaxAngle = practicalMaxAngle;
    }

    public String getDeviceName() {
        return deviceName;
    }

    /**
     * Unit is Degree
     */
    public double getAngle() {
        return servo.getAngle();
    }

    public class CustomRotateServoAction implements ActionEx {
        double toDegree;
        long nextActionTime;
        long oneStepTimeInMillSecond;
        double oneStepRotationInDegree;

        public CustomRotateServoAction(double toDegree, long oneStepTimeInMillSecond, double oneStepRotationInDegree) {
            this.toDegree = toDegree;
            this.oneStepTimeInMillSecond = oneStepTimeInMillSecond;
            this.oneStepRotationInDegree = oneStepRotationInDegree;

            nextActionTime = System.currentTimeMillis();
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long currentTime = System.currentTimeMillis();
            if (nextActionTime > currentTime) {
                return true;
            }
            nextActionTime = currentTime + oneStepTimeInMillSecond;

            double current = servo.getAngle();

            telemetry.addData(deviceName + " Current Position:", current);
            telemetry.addData(deviceName + " To Position:", toDegree);

            RobotLog.i(deviceName + " Current Position: %.1f, To: %.1f", current, toDegree);

            if (Math.abs(current - toDegree) <= 1) {
                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                return false;
            }

            if (Math.abs(current - toDegree) <= oneStepRotationInDegree) {
                servo.turnToAngle(toDegree);
            } else {
                int sign = toDegree > current ? 1 : -1;
                servo.rotateByAngle(oneStepRotationInDegree * sign);
            }

            return true;
        }
    }

    public class RotateWithJoystickServoAction implements ActionEx {
        Supplier<Float> joystickPosition;
        long nextActionTime;
        long cycleTimeInMillisecond;
        double maxRotateDegreeInOneSecond;

        boolean initialized = false;
        double rotateByAngle;

        public RotateWithJoystickServoAction(Supplier<Float> joystickPosition, long cycleTimeInMillisecond, double maxRotateDegreeInOneSecond) {
            this.joystickPosition = joystickPosition;
            this.cycleTimeInMillisecond = cycleTimeInMillisecond;
            this.maxRotateDegreeInOneSecond = maxRotateDegreeInOneSecond;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long currentTime = System.currentTimeMillis();
            double currentAngle = servo.getAngle(AngleUnit.DEGREES);

            if (!initialized) {
                nextActionTime = currentTime;
                initialized = true;
            }

            if (currentTime > nextActionTime) {
                rotateByAngle = maxRotateDegreeInOneSecond * cycleTimeInMillisecond / 1000.0 * joystickPosition.get();
                RobotLog.i(deviceName + " Current Angle: %.1f, rotateByAngle: %.1f", currentAngle, rotateByAngle);
                if (-1 < rotateByAngle && rotateByAngle < 1) {
                    servo.rotateBy(0); // stop rotate
                } else {
                    rotateByAngle = Range.clip(rotateByAngle, practicalMinAngle - currentAngle, practicalMaxAngle - currentAngle);
                    servo.rotateByAngle(rotateByAngle, AngleUnit.DEGREES);
                }

                nextActionTime += cycleTimeInMillisecond;
            }

            telemetry.addData(deviceName, " Current Angle:" + currentAngle + " rotateByAngle:" + rotateByAngle);
            return true;
        }
    }

    public class NormalRotateServoAction implements ActionEx {
        double toDegree;

        public NormalRotateServoAction(double toDegree) {
            this.toDegree = toDegree;
        }

        public String getDeviceName() {
            return deviceName;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.turnToAngle(toDegree);
            telemetry.addData(deviceName + " To Position:", toDegree);

            RobotLog.i(deviceName + " To Position: %.1f", toDegree);
            return true;
        }
    }


    /**
     * @param toDegree
     * @param oneStepTimeInMillSecond if you don't rotate in steps, set this to 0
     * @param oneStepRotationInDegree if you don't rotate in steps, set it to large number, such as 400
     */
    public ActionEx rotateCustom(double toDegree, long oneStepTimeInMillSecond, double oneStepRotationInDegree) {
        if (available)
            return new CustomRotateServoAction(toDegree, oneStepTimeInMillSecond, oneStepRotationInDegree);
        else
            return new ActionEx.NullActionEx();
    }

    public ActionEx rotateWithJoystick(Supplier<Float> joystickPosition, long cycleTimeInMillisecond, double maxRotateDegreeInOneSecond) {
        if (available)
            return new RotateWithJoystickServoAction(joystickPosition, cycleTimeInMillisecond, maxRotateDegreeInOneSecond);
        else
            return new ActionEx.NullActionEx();
    }

    public ActionEx rotateNormal(double toDegree) {
        if (available)
            return new NormalRotateServoAction(toDegree);
        else
            return new ActionEx.NullActionEx();
    }
}
