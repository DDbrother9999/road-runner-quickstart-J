package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import edu.nobles.robotics.motor.SlideMotor;

@TeleOp
public class SimpleFTCLibMotor extends LinearOpMode {

    private  MotorEx.Encoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        // Declare our motors
        // Make sure your ID's match your configuration
        MotorEx slideMotor = new MotorEx(hardwareMap, "backLeft");

        slideMotor.setInverted(false);
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotor.resetEncoder();
        encoder = slideMotor.encoder;

        slideMotor.setRunMode(Motor.RunMode.PositionControl);
        slideMotor.setPositionCoefficient(0.05);
        slideMotor.set(0);
        slideMotor.setPositionTolerance(15);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            double power = gamepadEx1.getLeftY();

            slideMotor.set(power);


            int current = slideMotor.getCurrentPosition();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                RobotLog.i(" Current Position: " + current);
                RobotLog.i("Motor Power: " + power);
                RobotLog.i("Motor Power: " + gamepadEx1.getLeftY());
            }

        }
    }
}