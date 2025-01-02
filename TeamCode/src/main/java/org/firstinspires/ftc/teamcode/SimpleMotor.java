package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SimpleMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorSlide0 = hardwareMap.dcMotor.get("motorSlide0");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = - gamepad1.left_stick_y; // Remember, Y stick value is reversed

            motorSlide0.setPower(power);
        }
    }
}