package org.firstinspires.ftc.teamcode;

import static edu.nobles.robotics.DeviceNameList.clawLeftName;
import static edu.nobles.robotics.DeviceNameList.clawRightName;
import static edu.nobles.robotics.DeviceNameList.intake1FlipName;
import static edu.nobles.robotics.DeviceNameList.intake1SlideExtendName;
import static edu.nobles.robotics.DeviceNameList.intake1SlideRetractName;
import static edu.nobles.robotics.DeviceNameList.intake1spinName;
import static edu.nobles.robotics.DeviceNameList.vertSlideLeftDownName;
import static edu.nobles.robotics.DeviceNameList.vertSlideLeftUpName;
import static edu.nobles.robotics.DeviceNameList.vertSlideRightDownName;
import static edu.nobles.robotics.DeviceNameList.vertSlideRightUpName;

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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.ActionEx;
import edu.nobles.robotics.motor.HorizontalExtender;
import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.servo.ServoDevice;

@TeleOp
@Config
public class CCCombinedActionTeleOpMode2 extends LinearOpMode {
    public static double move_XThrottle = 0.8;
    public static double move_YThrottle = 0.8;
    public static double move_RotateThrottle = 0.05;

    public static double intake1Flip_initDegree = 80;
    public static double intake1Flip_flatDegree = 220;

    public static long intake1Flip_m_cycleTime = 100;
    public static double intake1Flip_m_rotateInSec = 60;

    public static double claw1_openDegree = 121;
    public static double claw1_closeDegree = 88;
    public static double claw2_openDegree = 116;
    public static double claw2_closeDegree = 142;

    public static double intake1Spin_power = 0.5;

    public static int vertUp_max = 20000;
    public static float vertUp_maxPower = 0.6f;
    public static int vertUp_targetUp = 5000;
    public static int vertUp_targetDown = 500;

    public static int vertDown_max = 20000;
    public static double vertDown_maxPower = vertUp_maxPower;

    public static int vertDown_targetUp = -5000;
    public static int vertDown_targetDown = -500;

    public static double DownConstantPower = -0.00;


    //Vertical Slider's Position Controller
    public static double vertSlide_kP = 0.005;
    public static double vertSlide_positionTolerance = 15;   // allowed maximum error

    /**
     * factor of retracter power / extender power when extending
     */
    public static double intakeSlide_extendFactor = 0.8;
    /**
     * factor of extender power / retracter power when retracting
     */
    public static double intakeSlide_retractFactor = 1.2;

    public static double intakeSlide_joystickMaxPower = 0.5;

    public static double upPowerFactor = 0;
    public static double downPowerFactor = 1;

    private List<Action> runningActions = new ArrayList<>();

    private boolean flipFlat = false;

    private MecanumDrive mecanumDrive;
    private ServoDevice intake1FlipServo;
    private ServoDevice clawServo1;
    private ServoDevice clawServo2;
    private CRServo intake1SpinServo;
    private SlideMotor vertSlideUp;
    private SlideMotor vertSlideDown;
    private boolean vertSlideAlreadyStopped;

    private HorizontalExtender horizontalExtender;

    private GamepadEx gamepadEx1;
    private GamepadEx gamepadEx2;

    private long nextReportTime;

    private final List<String> unavailableHardwares = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // obtain a list of hubs
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        initHardware();

        addActionEx(intake1FlipServo.manualRotate(
                () -> {
                    if (gamepad1.dpad_up) return -1f;
                    if (gamepad1.dpad_down) return 1f;
                    if (gamepad2.dpad_up) return -1f;
                    if (gamepad2.dpad_down) return 1f;
                    return 0f;
                },
                intake1Flip_m_cycleTime,
                intake1Flip_m_rotateInSec));

        addActionEx(horizontalExtender.runWithJoystick(
                () -> {
                    if (gamepad1.left_trigger > 0.1) return gamepad1.left_trigger;
                    if (gamepad1.right_trigger > 0.1) return -gamepad1.right_trigger;
                    if (gamepad2.left_trigger > 0.1) return gamepad2.left_trigger;
                    if (gamepad2.right_trigger > 0.1) return -gamepad2.right_trigger;
                    return 0f;
                },
                intakeSlide_joystickMaxPower));

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // updated based on gamepads
            if (mecanumDrive.available) {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * move_XThrottle,
                                -gamepad1.left_stick_x * move_YThrottle),
                        -gamepad1.right_stick_x * move_RotateThrottle));
            }

            if (intake1SpinServo != null) {
                if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER) || gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    intake1SpinServo.set(-intake1Spin_power);
                } else if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER) || gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    intake1SpinServo.set(intake1Spin_power);
                } else {
                    intake1SpinServo.set(0);
                }
            }

//            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.BACK) && intake1FlipServo.available) {
//                RobotLog.i("Add flip0 action");
//                double toDegree = flipFlat ? intake1Flip_initDegree : intake1Flip_flatDegree;
//                addActionEx(intake1FlipServo.rotateCustom(toDegree, ServoActionTeleOpMode.intake1Flip_oneStepTime, ServoActionTeleOpMode.intake1Flip_oneStepRotation));
//                flipFlat = !flipFlat;
//            }

            vertSlideControl(-gamepad2.right_stick_y, false);
            vertSlideControl(-gamepad2.left_stick_y, true);
/*
            //DO NOT UNCOMMENT (Y already used below)
            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y) && vertSlideUp != null && vertSlideDown != null) {
                RobotLog.i("Add slide up");
                // vertSlideDown.zeroPowerWithFloat();
                addActionEx(vertSlideUp.moveSlide(vertUp_targetUp, vertUp_maxPower));
                //addActionEx(vertSlideDown.moveSlide(vertDown_targetUp, vertDown_maxPower));
            }
            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.X) && vertSlideUp != null && vertSlideDown != null) {
                RobotLog.i("Add slide down");
                // vertSlideUp.zeroPowerWithFloat();
                addActionEx(vertSlideUp.moveSlide(vertUp_targetDown, vertDown_maxPower));
                //addActionEx(vertSlideDown.moveSlide(vertDown_targetDown, vertDown_maxPower));
            }

 */

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.A)) {
                if (clawServo1.available)
                    clawServo1.servo.turnToAngle(claw1_openDegree, AngleUnit.DEGREES);
                if (clawServo2.available)
                    clawServo2.servo.turnToAngle(claw2_openDegree, AngleUnit.DEGREES);
            }

            if (gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)) {
                if (clawServo1.available)
                    clawServo1.servo.turnToAngle(claw1_closeDegree, AngleUnit.DEGREES);
                if (clawServo2.available)
                    clawServo2.servo.turnToAngle(claw2_closeDegree, AngleUnit.DEGREES);
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

        intake1FlipServo = new ServoDevice(intake1FlipName, hardwareMap, telemetry);
        intake1FlipServo.setPracticalAngles(intake1Flip_initDegree, intake1Flip_flatDegree);
        servoList.add(intake1FlipServo);

        clawServo1 = new ServoDevice(clawLeftName, hardwareMap, telemetry, 255);
        servoList.add(clawServo1);
        clawServo2 = new ServoDevice(clawRightName, hardwareMap, telemetry, 255);
        servoList.add(clawServo2);

        try {
            intake1SpinServo = new CRServo(hardwareMap, intake1spinName);
            intake1SpinServo.setInverted(true);
        } catch (Exception e) {
            RobotLog.e("intake1SpinServo is not available");
        }

        try {
            MotorEx vertSlideLeftUp = new MotorEx(hardwareMap, vertSlideLeftUpName, Motor.GoBILDA.RPM_435);
            vertSlideLeftUp.stopAndResetEncoder();
            //vertSlideLeftUp.setInverted(true);

            MotorEx vertSlideRightUp = new MotorEx(hardwareMap, vertSlideRightUpName, Motor.GoBILDA.RPM_435);
            vertSlideRightUp.stopAndResetEncoder();

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideUpGroup = new MotorGroupEx(vertSlideRightUp, vertSlideLeftUp);

            vertSlideUp = new SlideMotor(vertSlideUpGroup, telemetry, "vertSlideUp");
            vertSlideUp.setManualMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideUp are not available");
        }

        try {
            MotorEx vertSlideLeftDown = new MotorEx(hardwareMap, vertSlideLeftDownName, Motor.GoBILDA.RPM_312);
            vertSlideLeftDown.stopAndResetEncoder();
            //vertSlideLeftDown.setInverted(true);
            MotorEx vertSlideRightDown = new MotorEx(hardwareMap, vertSlideRightDownName, Motor.GoBILDA.RPM_312);
            vertSlideRightDown.stopAndResetEncoder();

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideDownGroup = new MotorGroupEx(vertSlideRightDown, vertSlideLeftDown);

            vertSlideDown = new SlideMotor(vertSlideDownGroup, telemetry, "vertSlideDown");
            vertSlideDown.setManualMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideDown are not available");
        }

        horizontalExtender = new HorizontalExtender(intake1SlideExtendName, intake1SlideRetractName, hardwareMap, telemetry);

        //GAMEPADS
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        if (!mecanumDrive.available) {
            unavailableHardwares.add("MecanumDrive");
        }
        if (vertSlideUp == null) {
            unavailableHardwares.add("vertSlideUp");
        }
        if (vertSlideDown == null) {
            unavailableHardwares.add("vertSlideDown");
        }
        if (intake1SpinServo == null) {
            unavailableHardwares.add("intake1SpinServo");
        }

        if (!horizontalExtender.available) unavailableHardwares.add("horizontalExtender");

        unavailableHardwares.addAll(servoList.stream().filter(servo -> !servo.available).map(ServoDevice::getDeviceName).collect(Collectors.toList()));
    }

    private void vertSlideControl(float power, boolean move) {
        if (vertSlideUp == null || vertSlideDown == null) return;

        boolean trueMoving = false;

        power = Range.clip(power, -vertUp_maxPower, vertUp_maxPower);

        if(move){
            if (Math.abs(power) < 0.01) {
                // Maintain tension
                //vertSlideDown.slideMotor.set(DownConstantPower);
                //telemetry.addData("Constant power: ",DownConstantPower);
            } else /* if (power > 0) */ {

                // slide up
                // vertSlideDown.zeroPowerWithFloat();

                int position = vertSlideUp.slideMotor.getCurrentPosition();
                //if (!(position > vertUp_max && power > 0)) {
                    // vertSlideUp.slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                    vertSlideUp.slideMotor.set(power);
                    if (power > 0) {
                        vertSlideDown.slideMotor.set(upPowerFactor * power);
                    } else if (power < 0) {
                        vertSlideDown.slideMotor.set(downPowerFactor * power);
                    }
                    trueMoving = true;
                //}
            }

        }
        else{
            vertSlideDown.slideMotor.set(power);
            trueMoving = true;
            vertSlideUp.slideMotor.set(0);
            vertSlideAlreadyStopped = true;
        }

        if (!trueMoving) {
            if (!vertSlideAlreadyStopped) {
                vertSlideUp.slideMotor.set(0);
                vertSlideDown.slideMotor.set(0);
                vertSlideAlreadyStopped = true;
            }
        } else {
            vertSlideAlreadyStopped = false;
        }
    }

    private void report(MecanumDrive drive, TelemetryPacket packet) {
        long currentTime = System.currentTimeMillis();

        telemetry.addData("Unavailable devices", String.join(", ", unavailableHardwares));

        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
//        telemetry.addData("right_stick_x", gamepad1.right_stick_x);

        if (drive.available) {
            if (currentTime > nextReportTime) {
                drive.updatePoseEstimate();
            }
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.round(Math.toDegrees(drive.pose.heading.toDouble())));
        }

        if (vertSlideUp != null) {
            telemetry.addData("vertSlideUp position", vertSlideUp.slideMotor.getCurrentPosition());
        }
        if (vertSlideDown != null) {
            telemetry.addData("vertSlideDown position", vertSlideDown.slideMotor.getCurrentPosition());
        }

        if (intake1SpinServo != null) {
            telemetry.addData("intake1Spin power", intake1SpinServo.get());
        }

        if (clawServo1.available) {
            telemetry.addData("clawServo1", clawServo1.getAngle());
        }

        if (clawServo2.available) {
            telemetry.addData("clawServo2", clawServo2.getAngle());
        }

        telemetry.update();

        packet.fieldOverlay().setStroke("#3F51B5");
        if (drive.available) {
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (currentTime > nextReportTime) {
            nextReportTime = currentTime + 500;
        }
    }

    private void addActionEx(ActionEx actionEx) {
        removeExistingAction(actionEx.getDeviceName());
        runningActions.add(actionEx);
    }

    private void removeExistingAction(String deviceName) {
        // Remove current action
        runningActions.removeIf(a -> a instanceof ActionEx && deviceName.equals(((ActionEx) a).getDeviceName()));
    }
}
