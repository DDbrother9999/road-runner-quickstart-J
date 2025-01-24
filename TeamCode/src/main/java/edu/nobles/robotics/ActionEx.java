package edu.nobles.robotics;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;

public interface ActionEx extends Action {
    String getDeviceName();

    public static class NullActionEx implements ActionEx {

        @Override
        public String getDeviceName() {
            return "";
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}

