package edu.nobles.robotics.servo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class ServoDevice {
    public static long sleepMillSecond = 100;

    public static double flip0_DegreeStep = 3; // servo rotates 'degreeStep' degree for every 'sleepMillSecond'
    public static double flip0_InitDegree = 0;
    public static double flip0_FlatDegree = 90;

    private final String deviceName;
    private final SimpleServo servo;
    private final Telemetry telemetry;

    public ServoDevice(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        this.deviceName = deviceName;
        servo = new SimpleServo(hardwareMap, deviceName, 0, 180, AngleUnit.DEGREES);
        this.telemetry = telemetry;
    }

    /**
     * Unit is Degree
     */
    public double getAngle() {
        return servo.getAngle();
    }

    public class RotateServoAction implements Action {
        double toDegree;
        long nextActionTime;

        public RotateServoAction(double toDegree) {
            this.toDegree = toDegree;
            nextActionTime = System.currentTimeMillis();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long currentTime = System.currentTimeMillis();
            if (nextActionTime > currentTime) {
                return true;
            }
            nextActionTime = currentTime + sleepMillSecond;

            double current = servo.getAngle();

            telemetry.addData(deviceName + " Current Position:", current);
            telemetry.addData(deviceName + " To Position:", toDegree);

            RobotLog.i(deviceName + " Current Position: %.1f, To: %.1f", current, toDegree);

            if (Math.abs(current - toDegree) <= 1) {
                RobotLog.i(deviceName + " Stop");
                telemetry.addLine(deviceName + " Stop");
                telemetry.update();
                return false;
            }
            telemetry.update();

            if (Math.abs(current - toDegree) <= flip0_DegreeStep) {
                servo.turnToAngle(toDegree);
            } else {
                int sign = toDegree > current ? 1 : -1;
                servo.rotateByAngle(flip0_DegreeStep * sign);
            }

            return true;
        }
    }

    public Action rotate(double toDegree) {
        return new RotateServoAction(toDegree);
    }
}
