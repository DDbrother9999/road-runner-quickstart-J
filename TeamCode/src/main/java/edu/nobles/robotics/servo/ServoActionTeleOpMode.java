package edu.nobles.robotics.servo;

import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.*;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

import edu.nobles.robotics.servo.ServoDevice.CustomRotateServoAction;

@TeleOp
@Config
public class ServoActionTeleOpMode extends LinearOpMode {
    public static double xThrottle = 0.4;
    public static double yThrottle = 0.4;
    public static double headThrottle = 0.05;
    // if you don't rotate in steps, set this to 0
    public static long intake1Flip_oneStepTime = 0;
    // if you don't rotate in steps, set it to large number, such as 400
    public static double intake1Flip_oneStepRotation = 400;

    private List<Action> runningActions = new ArrayList<>();

    private boolean flipFlat = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        ServoDevice flipServo = new ServoDevice("flip0", hardwareMap, telemetry);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // updated based on gamepads
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * xThrottle,
                            -gamepad1.left_stick_x * yThrottle
                    ),
                    -gamepad1.right_stick_x * headThrottle
            ));

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                RobotLog.i("Add Flip action");
                // Remove current Flip action
                runningActions.removeIf(a -> a instanceof CustomRotateServoAction
                        && flipServo.getDeviceName().equals(((CustomRotateServoAction) a).getDeviceName()));
                runningActions.add(flipServo.rotateCustom(flipFlat ? intake1Flip_initDegree : intake1Flip_flatDegree,
                        intake1Flip_oneStepTime, intake1Flip_oneStepRotation));
                flipFlat = !flipFlat;
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
