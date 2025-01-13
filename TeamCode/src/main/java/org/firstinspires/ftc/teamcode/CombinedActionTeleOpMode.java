package org.firstinspires.ftc.teamcode;

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

import java.util.ArrayList;
import java.util.List;

import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.parameters.ParameterManager;
import edu.nobles.robotics.servo.ServoDevice;
import edu.nobles.robotics.servo.ServoDevice.RotateServoAction;

@TeleOp
@Config
public class CombinedActionTeleOpMode extends LinearOpMode {
    public static double xThrottle = 0.4;
    public static double yThrottle = 0.4;
    public static double headThrottle = 0.05;

    private List<Action> runningActions = new ArrayList<>();

    private boolean flipFlat = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        // ServoDevice flipServo = new ServoDevice("flip0", hardwareMap, telemetry);

        // HANDLE INVERTED MOTORS HERE
        MotorEx vertSlideLeftUp = new MotorEx(hardwareMap, "vertSlideLeftUp", Motor.GoBILDA.RPM_223);
        MotorEx vertSlideRightUp = new MotorEx(hardwareMap, "vertSlideRightUp", Motor.GoBILDA.RPM_435);
        MotorEx vertSlideLeftDown = new MotorEx(hardwareMap, "vertSlideLeftDown", Motor.GoBILDA.RPM_223);
        MotorEx vertSlideRightDown = new MotorEx(hardwareMap, "vertSlideRightDown", Motor.GoBILDA.RPM_435);


        //DON'T INVERT MOTORS AFTER HERE
        MotorGroup vertSlideUpGroup = new MotorGroup(vertSlideLeftUp, vertSlideRightUp);
        MotorGroup vertSlideDownGroup = new MotorGroup(vertSlideLeftDown, vertSlideRightDown);

        SlideMotor vertSlideUp = new SlideMotor(vertSlideUpGroup, telemetry, "vertSlideUp");
        SlideMotor vertSlideDown = new SlideMotor(vertSlideDownGroup, telemetry, "vertSlideDown");

        //GAMEPADS
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

            /*
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                RobotLog.i("Add Flip action");
                // Remove current Flip action
                runningActions.removeIf(a -> a instanceof RotateServoAction
                        && flipServo.getDeviceName().equals(((RotateServoAction) a).getDeviceName()));
                runningActions.add(flipServo.rotate(flipFlat ? ServoDevice.flip0_InitDegree : ServoDevice.flip0_FlatDegree));
                flipFlat = !flipFlat;
            }

             */

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                RobotLog.i("Add extend action");
                // Remove current actions on motors
                runningActions.removeIf(a -> a instanceof SlideMotor.PosMoveSlideAction
                        && vertSlideUp.getDeviceName().equals(((SlideMotor.PosMoveSlideAction) a).getDeviceName()));
                runningActions.removeIf(a -> a instanceof SlideMotor.PosMoveSlideAction
                        && vertSlideDown.getDeviceName().equals(((SlideMotor.PosMoveSlideAction) a).getDeviceName()));

                runningActions.add(vertSlideUp.PosMoveSlide(ParameterManager.targetVertUpExtend, ParameterManager.maxVertUpPower));
                runningActions.add(vertSlideDown.PosMoveSlide(ParameterManager.targetVertDownExtend, ParameterManager.maxVertUpPower));
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                RobotLog.i("Add retract action");
                // Remove current actions on motors
                runningActions.removeIf(a -> a instanceof SlideMotor.PosMoveSlideAction
                        && vertSlideUp.getDeviceName().equals(((SlideMotor.PosMoveSlideAction) a).getDeviceName()));
                runningActions.removeIf(a -> a instanceof SlideMotor.PosMoveSlideAction
                        && vertSlideDown.getDeviceName().equals(((SlideMotor.PosMoveSlideAction) a).getDeviceName()));
                runningActions.add(vertSlideUp.PosMoveSlide(ParameterManager.targetVertUpRetract, ParameterManager.maxVertUpPower));
                runningActions.add(vertSlideDown.PosMoveSlide(ParameterManager.targetVertDownRetract, ParameterManager.maxVertDownPower));
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


    //UNUSED
    private void removePreviousActionSlideMotor(Action action, SlideMotor slideMotor){
        runningActions.removeIf(a -> a instanceof SlideMotor.PosMoveSlideAction
                && slideMotor.getDeviceName().equals(((SlideMotor.PosMoveSlideAction) a).getDeviceName()));
    }
}
