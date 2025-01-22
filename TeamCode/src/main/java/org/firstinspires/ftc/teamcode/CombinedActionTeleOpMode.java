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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import edu.nobles.robotics.ActionEx;
import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.servo.ServoDevice;

@TeleOp
@Config
public class CombinedActionTeleOpMode extends LinearOpMode {
    public static boolean useGamepadForMoving = false;

    public static double moveXThrottle = 0.4;
    public static double moveYThrottle = 0.4;
    public static double moveRotateThrottle = 0.05;

    public static double flip0_InitDegree = 0;
    public static double flip0_FlatDegree = 300;

    public static double maxVertUpPower = 0.25;
    public static double maxVertDownPower = maxVertUpPower;
    public static int targetVertUpExtend = 2000;
    public static int targetVertDownExtend = targetVertUpExtend;
    public static int targetVertUpRetract = 0;
    public static int targetVertDownRetract = targetVertUpRetract;

    public static int vertSlideUpMax = 2000;
    public static int vertSlideDownMax = 2000;

    //Position Controller
    public static double vertSlide_kP = 0.05;
    public static double vertSlidePositionTolerance = 15;   // allowed maximum error

    private List<Action> runningActions = new ArrayList<>();

    private boolean flipFlat = false;
    private boolean flipFlat2 = false;
    private MecanumDrive mecanumDrive;
    private ServoDevice flipServo;
    private ServoDevice servoArmSpinner;
    private SlideMotor vertSlideUp;
    private SlideMotor vertSlideDown;
    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // obtain a list of hubs
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initHareware();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // updated based on gamepads
            if (useGamepadForMoving) {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * moveXThrottle,
                                -gamepad1.left_stick_x * moveYThrottle
                        ),
                        -gamepad1.right_stick_x * moveRotateThrottle
                ));
            } else {
                vertSlideControl();
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                RobotLog.i("Add flip0 action");
                addActionEx(flipServo.rotate(flipFlat ? flip0_InitDegree : flip0_FlatDegree));
                flipFlat = !flipFlat;
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                RobotLog.i("Add slide up");
                vertSlideUp.setActionMode();
                vertSlideDown.zeroPowerWithFloat();
                addActionEx(vertSlideUp.moveSlide(targetVertUpExtend, maxVertUpPower));
                // addActionEx(vertSlideDown.moveSlide(ParameterManager.targetVertDownExtend, ParameterManager.maxVertUpPower));
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                RobotLog.i("Add slide down");
                vertSlideDown.setActionMode();
                vertSlideUp.zeroPowerWithFloat();
                // addActionEx(vertSlideUp.moveSlide(ParameterManager.targetVertUpRetract, ParameterManager.maxVertUpPower));
                addActionEx(vertSlideDown.moveSlide(targetVertDownRetract, maxVertDownPower));
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

            report(mecanumDrive, packet);
        }
    }

    private void initHareware() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        flipServo = new ServoDevice("servoArmFlip", hardwareMap, telemetry);
        //servoArmSpinner = new ServoDevice("servoArmSpinner", hardwareMap, telemetry);

        // HANDLE INVERTED MOTORS HERE
        MotorEx vertSlideLeftUp = new MotorEx(hardwareMap, "vertSlideLeftUp", Motor.GoBILDA.RPM_223);
        vertSlideLeftUp.setInverted(true);
        MotorEx vertSlideRightUp = new MotorEx(hardwareMap, "vertSlideRightUp", Motor.GoBILDA.RPM_223);

        MotorEx vertSlideLeftDown = new MotorEx(hardwareMap, "vertSlideLeftDown", Motor.GoBILDA.RPM_435);
        vertSlideLeftDown.setInverted(true);
        MotorEx vertSlideRightDown = new MotorEx(hardwareMap, "vertSlideRightDown", Motor.GoBILDA.RPM_435);

        //DON'T INVERT MOTORS AFTER HERE
        MotorGroupEx vertSlideUpGroup = new MotorGroupEx(vertSlideRightUp, vertSlideLeftUp);
        MotorGroupEx vertSlideDownGroup = new MotorGroupEx(vertSlideRightDown, vertSlideLeftDown);

        vertSlideUp = new SlideMotor(vertSlideUpGroup, telemetry, "vertSlideUp");
        vertSlideUp.setManualMode();
        vertSlideDown = new SlideMotor(vertSlideDownGroup, telemetry, "vertSlideDown");
        vertSlideDown.setManualMode();

        //GAMEPADS
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
    }

    private void vertSlideControl() {
        boolean moving = false;
        float upPower = -gamepad1.left_stick_y;
        if (upPower > 0) {
            // slide up
            vertSlideDown.zeroPowerWithFloat();

            if (vertSlideUp.slideMotor.getCurrentPosition() < vertSlideUpMax) {
                vertSlideUp.slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                vertSlideUp.slideMotor.set(upPower);
                moving = true;
            }
        } else if (upPower < 0) {
            // slide down
            vertSlideUp.zeroPowerWithFloat();
            if (vertSlideDown.slideMotor.getCurrentPosition() < vertSlideDownMax) {
                vertSlideDown.slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                vertSlideDown.slideMotor.set(-upPower);
                moving = true;
            }
        }

        if (!moving) {
            vertSlideUp.slideMotor.set(0);
            vertSlideDown.slideMotor.set(0);
        }
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        telemetry.addData("vertSlide up position", vertSlideUp.slideMotor.getCurrentPosition());
        telemetry.addData("vertSlide down position", vertSlideDown.slideMotor.getCurrentPosition());

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

    private void addActionEx(ActionEx actionEx) {
        removeExistingAction(actionEx.getDeviceName());
        runningActions.add(actionEx);
    }

    private void removeExistingAction(String deviceName) {
        // Remove current action
        runningActions.removeIf(a -> a instanceof ActionEx
                && deviceName.equals(((ActionEx) a).getDeviceName()));
    }
}
