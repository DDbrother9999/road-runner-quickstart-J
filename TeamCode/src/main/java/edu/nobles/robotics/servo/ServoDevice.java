package edu.nobles.robotics.servo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import edu.nobles.robotics.ActionEx;

@Config
public class ServoDevice {
    public static long sleepMillSecond = 0;

    public static double flip0_DegreeStep = 10; // servo rotates 'degreeStep' degree for every 'sleepMillSecond'

    private final String deviceName;
    private SimpleServo servo;
    private final Telemetry telemetry;

    public boolean available = true;

    public ServoDevice(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        this.deviceName = deviceName;
        this.telemetry = telemetry;
        try {
            servo = new SimpleServo(hardwareMap, deviceName, 0, 300, AngleUnit.DEGREES); //All gobilda servos are 0 to 300
        } catch (Exception e) {
            available = false;
            RobotLog.e("Servo "+deviceName+" is not available");
            return;
        }


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

    public class RotateServoAction implements ActionEx {
        double toDegree;
        long nextActionTime;

        public RotateServoAction(double toDegree) {
            this.toDegree = toDegree;
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

    public ActionEx rotate(double toDegree) {
        return new RotateServoAction(toDegree);
    }
}
