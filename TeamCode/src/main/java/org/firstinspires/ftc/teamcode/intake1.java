package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config

public class intake1{
    private ServoEx flip0;

    public intake1(HardwareMap hardwareMap) {
        flip0 = new SimpleServo(
                hardwareMap, "flip0", 0, 300,
                AngleUnit.DEGREES
        );
    }



    public class flip200 implements Action {

        public boolean run(@NonNull TelemetryPacket packet) {
            flip0.turnToAngle(200);
            return false;
        }

    }

    public Action flip200() {
        return new flip200();
    }

    public class flip250 implements Action {

        public boolean run(@NonNull TelemetryPacket packet) {
            flip0.turnToAngle(300);
            return false;
        }

    }

    public Action flip250() {
        return new flip250();
    }
}