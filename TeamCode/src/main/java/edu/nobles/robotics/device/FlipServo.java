package edu.nobles.robotics.device;

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
public class FlipServo {
    public static long sleepMillSecond = 100;
    public static double degreeStep = 3;
    public static double initDegree = 0;
    public static double flatDegree = 90;

    private SimpleServo fliper;

    private Telemetry telemetry;

    public FlipServo(HardwareMap hardwareMap, Telemetry telemetry) {
        fliper = new SimpleServo(hardwareMap, "flip0", 0, 180, AngleUnit.DEGREES);
        this.telemetry = telemetry;
    }

    public class RotateFlipAction implements Action {
        double toDegree;
        long nextActionTime;

        public RotateFlipAction(double toDegree) {
            this.toDegree = toDegree;
            nextActionTime = System.currentTimeMillis();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long currentTime = System.currentTimeMillis();
            if(nextActionTime > currentTime) {
                return true;
            }
            nextActionTime = currentTime + sleepMillSecond;

            double current = fliper.getAngle();
            packet.put("Flip Current Position", current);
            packet.put("Flip To Position", toDegree);
            telemetry.addData("Flip Current Position:",current);
            telemetry.addData("Flip To Position:",toDegree);

            RobotLog.i("Flip Current Position: %f", current);
            RobotLog.i("Flip To Position: %f", toDegree);

            if (Math.abs(current - toDegree) <= 1) {
                packet.addLine("Flip Stop");
                RobotLog.i("Flip Stop");
                telemetry.addLine("Flip Stop");
                telemetry.update();
                return false;
            }
            telemetry.update();

            if (Math.abs(current - toDegree) <= degreeStep) {
                fliper.turnToAngle(toDegree);
            } else {
                int sign = toDegree > current ? 1 : -1;
                fliper.rotateByAngle( degreeStep *  sign);
            }

            return true;
        }
    }

    public Action rotate(double toDegree) {
        return new RotateFlipAction(toDegree);
    }
}
