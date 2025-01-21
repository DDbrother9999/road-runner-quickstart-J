package edu.nobles.robotics.sensors;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config

@TeleOp
public class SimpleMagnetSwitch extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TouchSensor MagnetSwitch = hardwareMap.get(TouchSensor.class, "magnet");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Limit Switch Val", MagnetSwitch.getValue());
            packet.put("Limit Switch Pushed?", MagnetSwitch.isPressed());

            packet.put("status", "alive");
            dashboard.sendTelemetryPacket(packet);

        }
    }
}