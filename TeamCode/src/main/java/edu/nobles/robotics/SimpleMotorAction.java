package edu.nobles.robotics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.motor.SlideMotor.PosMoveSlideAction;


@TeleOp
@Config


public class SimpleMotorAction extends LinearOpMode {

    private boolean extended = false;

    public static double xThrottle = 0.4;
    public static double yThrottle = 0.4;
    public static double headThrottle = 0.05;
    public static String motorName = "vertSlideLeftUp";

    private List<Action> runningActions = new ArrayList<>();
    private MotorGroupEx motorGroupSolo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MotorEx simpleMotor = new MotorEx(hardwareMap, motorName);
        motorGroupSolo = new MotorGroupEx(simpleMotor);
        SlideMotor motorSlide0 = new SlideMotor(motorGroupSolo, telemetry, "simpleMotor");
        motorSlide0.setManualMode();
        motorGroupSolo.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();


            motorSlide0.slideMotor.set(-gamepad1.left_stick_y);

            // updated based on gamepads
            /*
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * xThrottle,
                            -gamepad1.left_stick_x * yThrottle
                    ),
                    -gamepad1.right_stick_x * headThrottle
            ));

             */

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                RobotLog.i("Add extend action");
                // Remove current extend action
                runningActions.removeIf(a -> a instanceof PosMoveSlideAction
                        && motorSlide0.getDeviceName().equals(((PosMoveSlideAction) a).getDeviceName()));
                runningActions.add(motorSlide0.moveSlide(extended ? 0 : 2000, 0.25));
                extended=!extended;
            }

            // update running actions
            if (!runningActions.isEmpty()) {
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;
            }

            report(drive, packet);
        }
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        telemetry.addData("vertSlide position", motorGroupSolo.getCurrentPosition());

        drive.updatePoseEstimate();

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
