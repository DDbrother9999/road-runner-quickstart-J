package edu.nobles.robotics.sensors;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


// @TeleOp
public class SimpleDistanceSensor extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        SensorRevTOFDistance distanceSensor = new SensorRevTOFDistance(hardwareMap, "distanceSensor1");
        waitForStart();

        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }

    }

}
