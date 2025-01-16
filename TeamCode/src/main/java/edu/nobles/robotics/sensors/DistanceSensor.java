package edu.nobles.robotics.sensors;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistanceSensor{

    private final SensorRevTOFDistance distanceSensor;
    private final String deviceName;
    private final Telemetry telemetry;

    public DistanceSensor(String deviceName, SensorRevTOFDistance distanceSensor, Telemetry telemetry){
        this.distanceSensor=distanceSensor;
        this.deviceName = deviceName;
        this.telemetry = telemetry;
    }


    public String getDeviceName() {
        return deviceName;
    }

    /**
     * Unit is CM
     */
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}

