package edu.nobles.robotics.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

@TeleOp
public class TestSimpleCRServo extends LinearOpMode {
    public static String CRservoName = "servoArmSpinner";


    @Override
    public void runOpMode() throws InterruptedException {

        CRServo flip0 = new CRServo(
                hardwareMap, CRservoName
        );
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double triggerSpin = gamepad1.left_stick_y;
            flip0.set(triggerSpin);
            telemetry.addData("triggerSpin: ",triggerSpin);
            telemetry.update();

        }
    }
}