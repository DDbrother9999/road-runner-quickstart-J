package edu.nobles.robotics.servo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//Very simple servo test
public class intake1{
    private ServoEx flipServo;

    public intake1(HardwareMap hardwareMap) {
        flipServo = new SimpleServo(
                hardwareMap, "flipServo", 0, 300,
                AngleUnit.DEGREES
        );
    }



    public class flip300 implements Action {

        public boolean run(@NonNull TelemetryPacket packet) {
            flipServo.turnToAngle(300);
            return false;
        }

    }

    public Action flip300() {
        return new flip300();
    }

    public class flip0 implements Action {

        public boolean run(@NonNull TelemetryPacket packet) {
            flipServo.turnToAngle(0);
            return false;
        }

    }

    public Action flip0() {
        return new flip0();
    }
}