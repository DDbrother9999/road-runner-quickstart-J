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
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.ActionEx;
import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.servo.ServoDevice;

@TeleOp
@Config
public class CombinedActionTeleOpMode extends LinearOpMode {
    public static boolean useGamepadForMoving = false;

    public static double move_XThrottle = 0.4;
    public static double move_YThrottle = 0.4;
    public static double move_RotateThrottle = 0.05;

    public static double flip0_initDegree = 0;
    public static double flip0_flatDegree = 300;

    // if you don't rotate in steps, set this to 0
    public static long flip0_oneStepTimeInMillSecond = 0;
    // if you don't rotate in steps, set it to large number, such as 400
    public static double flip0_oneStepRotationInDegree = 400;

    public static double claw1_openDegree = 0;
    public static double claw1_closeDegree = 30;
    public static double claw2_openDegree = 30;
    public static double claw2_closeDegree = 0;

    public static double contRotateServo_power = 0.25;


    public static int vertUp_max = 2000;
    public static double vertUp_maxPower = 0.25;
    public static int vertUp_targetUp = 1000;
    public static int vertUp_targetDown = 0;

    public static int vertDown_max = 2000;
    public static double vertDown_maxPower = vertUp_maxPower;
    public static int vertDown_targetUp = -1000;
    public static int vertDown_targetDown = 0;


    //Vertical Slider's Position Controller
    public static double vertSlide_kP = 0.05;
    public static double vertSlide_positionTolerance = 15;   // allowed maximum error

    private List<Action> runningActions = new ArrayList<>();

    private boolean flipFlat = false;
    private boolean clawOpen = true;
    private boolean contRotateStoped = true;

    private MecanumDrive mecanumDrive;
    private ServoDevice flipServo;
    private ServoDevice clawServo1;
    private ServoDevice clawServo2;
    private CRServo contRotateServo;
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

        initHardware();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // updated based on gamepads
            if (useGamepadForMoving && mecanumDrive.available) {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * move_XThrottle,
                                -gamepad1.left_stick_x * move_YThrottle
                        ),
                        -gamepad1.right_stick_x * move_RotateThrottle
                ));
            } else {
                vertSlideControl();
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B) && flipServo.available) {
                RobotLog.i("Add flip0 action");
                double toDegree = flipFlat ? flip0_initDegree : flip0_flatDegree;
                addActionEx(flipServo.rotateCustom(toDegree, flip0_oneStepTimeInMillSecond, flip0_oneStepRotationInDegree));
                flipFlat = !flipFlat;
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y) && vertSlideUp != null && vertSlideDown != null) {
                RobotLog.i("Add slide up");
                // vertSlideDown.zeroPowerWithFloat();
                addActionEx(vertSlideUp.moveSlide(vertUp_targetUp, vertUp_maxPower));
                addActionEx(vertSlideDown.moveSlide(vertDown_targetUp, vertDown_maxPower));
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X) && vertSlideUp != null && vertSlideDown != null) {
                RobotLog.i("Add slide down");
                // vertSlideUp.zeroPowerWithFloat();
                addActionEx(vertSlideUp.moveSlide(vertUp_targetDown, vertUp_maxPower));
                addActionEx(vertSlideDown.moveSlide(vertDown_targetDown, vertDown_maxPower));
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && clawServo1.available && clawServo2.available) {
                if (clawOpen) {
                    clawServo1.servo.turnToAngle(claw1_closeDegree);
                    clawServo2.servo.turnToAngle(claw2_closeDegree);
                } else {
                    clawServo1.servo.turnToAngle(claw1_openDegree);
                    clawServo2.servo.turnToAngle(claw2_openDegree);
                }
                clawOpen = !clawOpen;
            }

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && contRotateServo != null) {
                if (contRotateStoped) {
                    contRotateServo.set(contRotateServo_power);
                } else {
                    contRotateServo.set(0);
                }
                contRotateStoped = !contRotateStoped;
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

    private void initHardware() {
        List<ServoDevice> servoList = new ArrayList<>();

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        flipServo = new ServoDevice("servoArmFlip", hardwareMap, telemetry);
        servoList.add(flipServo);
        //servoArmSpinner = new ServoDevice("servoArmSpinner", hardwareMap, telemetry);
        clawServo1 = new ServoDevice("claw1", hardwareMap, telemetry);
        servoList.add(clawServo1);
        clawServo2 = new ServoDevice("claw2", hardwareMap, telemetry);
        servoList.add(clawServo2);
        try {
            contRotateServo = new CRServo(hardwareMap, "contRotate");
        } catch (Exception e) {
            RobotLog.e("Continuous Rotate Servo is not available");
        }

        try {
            MotorEx vertSlideLeftUp = new MotorEx(hardwareMap, "vertSlideLeftUp", Motor.GoBILDA.RPM_435);
            vertSlideLeftUp.setInverted(true);
            MotorEx vertSlideRightUp = new MotorEx(hardwareMap, "vertSlideRightUp", Motor.GoBILDA.RPM_435);

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideUpGroup = new MotorGroupEx(vertSlideRightUp, vertSlideLeftUp);

            vertSlideUp = new SlideMotor(vertSlideUpGroup, telemetry, "vertSlideUp");
            vertSlideUp.setManualMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideUp are not available");
        }

        try {
            MotorEx vertSlideLeftDown = new MotorEx(hardwareMap, "vertSlideLeftDown", Motor.GoBILDA.RPM_312);
            vertSlideLeftDown.setInverted(true);
            MotorEx vertSlideRightDown = new MotorEx(hardwareMap, "vertSlideRightDown", Motor.GoBILDA.RPM_312);

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideDownGroup = new MotorGroupEx(vertSlideRightDown, vertSlideLeftDown);

            vertSlideDown = new SlideMotor(vertSlideDownGroup, telemetry, "vertSlideDown");
            vertSlideDown.setManualMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideDown are not available");
        }

        //GAMEPADS
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        List<String> unavailableHardwares = new ArrayList<>();
        if (!mecanumDrive.available) {
            unavailableHardwares.add("MecanumDrive");
        }
        if (vertSlideUp == null) {
            unavailableHardwares.add("vertSlideUp");
        }
        if (vertSlideDown == null) {
            unavailableHardwares.add("vertSlideDown");
        }
        if (contRotateServo == null) {
            unavailableHardwares.add("contRotateServo");
        }
        unavailableHardwares.addAll(
                servoList.stream().filter(servo -> !servo.available).map(ServoDevice::getDeviceName).collect(Collectors.toList()));

        telemetry.addData("Unavailable devices", String.join(", ", unavailableHardwares));
    }

    private void vertSlideControl() {
        boolean moving = false;
        float upPower = -gamepad1.left_stick_y;
        if (upPower > 0) {
            // slide up
            vertSlideDown.zeroPowerWithFloat();

            if (vertSlideUp.slideMotor.getCurrentPosition() < vertUp_max) {
                vertSlideUp.slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                vertSlideUp.slideMotor.set(upPower);
                moving = true;
            }
        } else if (upPower < 0) {
            // slide down
            vertSlideUp.zeroPowerWithFloat();
            if (vertSlideDown.slideMotor.getCurrentPosition() < vertDown_max) {
                vertSlideDown.slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                vertSlideDown.slideMotor.set(-upPower);
                moving = true;
            }
        }

        if (!moving) {
            vertSlideUp.slideMotor.set(0);
            vertSlideDown.slideMotor.set(0);
        }
        telemetry.addData("vertSlideUp position: ", vertSlideUp.slideMotor.getCurrentPosition());
        telemetry.addData("vertSlideDown position: ", vertSlideDown.slideMotor.getCurrentPosition());
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        telemetry.addData("vertSlide up position", vertSlideUp.slideMotor.getCurrentPosition());
        telemetry.addData("vertSlide down position", vertSlideDown.slideMotor.getCurrentPosition());

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
//        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        if (drive.available) {
            drive.updatePoseEstimate();
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        }
        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        if (drive.available) {
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
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
