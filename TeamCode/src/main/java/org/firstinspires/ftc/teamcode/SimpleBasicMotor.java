package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class SimpleBasicMotor extends LinearOpMode {
    public static String MotorName = "vertSlideRightDown";
    @Override
    public void runOpMode() throws InterruptedException {


        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorSlide0 = hardwareMap.dcMotor.get(MotorName);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = - gamepad1.left_stick_y; // Remember, Y stick value is reversed

            motorSlide0.setPower(power);
        }
    }
}