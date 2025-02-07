package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.intake1Flip_flatDegree;
import static org.firstinspires.ftc.teamcode.ANewActionTeleOpMode.intake1Flip_initDegree;
import static edu.nobles.robotics.parameters.DeviceNameList.clawName;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1FlipName;
import static edu.nobles.robotics.parameters.DeviceNameList.intake1spinName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideDownName;
import static edu.nobles.robotics.parameters.DeviceNameList.vertSlideUpName;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.ANewActionTeleOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.parameters.ParameterManager;
import edu.nobles.robotics.servo.ServoDevice;


@Config
@Autonomous(name = "FinalAuto", group = "Autonomous")
public class FinalAuto extends LinearOpMode {
    public static double startX= 36;
    public static double startY= -62.5;
    public static double startHeading= -90;

    public static double firstX= -90;
    public static double targetY= -90;

    //start pose
    private Pose2d beginPose = new Pose2d(14, -63, Math.toRadians(-90));

    private MecanumDrive mecanumDrive;
    private ServoDevice intake1FlipServo;
    private ServoDevice clawServo;
    private CRServo intake1SpinServo;
    private SlideMotor vertSlideUp;
    private SlideMotor vertSlideDown;
    private boolean vertSlideAlreadyStopped;

    private SlideMotor horizontalExtender;

    private final List<String> unavailableHardwares = new ArrayList<>();

    @Override
    public void runOpMode() {

        initHardware();



        TrajectoryActionBuilder moveInitialTraj = mecanumDrive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0, -43))
                //Check for vertical slide extension
                .strafeTo(new Vector2d(0, -34));

        TrajectoryActionBuilder moveToSubTraj = moveInitialTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -34));

        TrajectoryActionBuilder pushTraj = moveToSubTraj.endTrajectory().fresh()
                .strafeTo(new Vector2d(36, -34))
                .strafeTo(new Vector2d(36, 0))
                .strafeTo(new Vector2d(47, 0))
                .strafeTo(new Vector2d(47, -60)) //Drops off in observation zone
                .strafeTo(new Vector2d(47, 0))
                .strafeTo(new Vector2d(56, 0))
                .strafeTo(new Vector2d(56, -60));

                /*
                .strafeTo(new Vector2d(56, 0))
                .strafeTo(new Vector2d(65, 0))
                .strafeTo(new Vector2d(65, -60));

                 */

        Action moveInitial = moveInitialTraj.build();
        Action moveToSub = moveToSubTraj.build();
        Action push = pushTraj.build();

        // actions that need to happen on init; for instance, a claw tightening.
        // INSERT HERE

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                moveInitial,
                                vertSlideUp.moveSlide(ParameterManager.highChamberExtendUpPos, ANewActionTeleOpMode.vertUp_maxPower),
                                vertSlideDown.moveSlide(ParameterManager.highChamberExtendDownPos, ANewActionTeleOpMode.vertDown_maxPower)
                        ),
                        moveToSub,
                        new SleepAction(1.0),
                        new ParallelAction(
                                vertSlideUp.moveSlide(ParameterManager.highChamberRetractUpPos, ANewActionTeleOpMode.vertUp_maxPower),
                                vertSlideDown.moveSlide(ParameterManager.highChamberRetractDownPos, ANewActionTeleOpMode.vertDown_maxPower)
                        ),

                        //TODO: May need race action (new, need to merge) with timer to open claw even if motors stall
                        clawServo.rotateNormal(ANewActionTeleOpMode.claw1_openDegree),
                        new SleepAction(1.0),
                        new ParallelAction(
                                vertSlideUp.moveSlide(ParameterManager.highChamberExtendUpPos, ANewActionTeleOpMode.vertUp_maxPower),
                                vertSlideDown.moveSlide(ParameterManager.highChamberExtendDownPos, ANewActionTeleOpMode.vertDown_maxPower)
                        ),
                        push
                )
        );
    }

    private void initHardware() {
        List<ServoDevice> servoList = new ArrayList<>();

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake1FlipServo = new ServoDevice(intake1FlipName, hardwareMap, telemetry);
        intake1FlipServo.setPracticalAngles(intake1Flip_initDegree, intake1Flip_flatDegree);
        servoList.add(intake1FlipServo);

        clawServo = new ServoDevice(clawName, hardwareMap, telemetry, 255);
        servoList.add(clawServo);

        try {
            intake1SpinServo = new CRServo(hardwareMap, intake1spinName);
            intake1SpinServo.setInverted(true);
        } catch (Exception e) {
            RobotLog.e("intake1SpinServo is not available");
        }

        try {
            MotorEx vertSlideLeftUp = new MotorEx(hardwareMap, vertSlideUpName, Motor.GoBILDA.RPM_435);
            vertSlideLeftUp.stopAndResetEncoder();
            //vertSlideLeftUp.setInverted(true);

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideUpGroup = new MotorGroupEx(vertSlideLeftUp);

            vertSlideUp = new SlideMotor(vertSlideUpGroup, telemetry, "vertSlideUp");
            vertSlideUp.setActionMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideUp are not available");
        }

        try {
            MotorEx vertSlideRightDown = new MotorEx(hardwareMap, vertSlideDownName, Motor.GoBILDA.RPM_312);
            vertSlideRightDown.stopAndResetEncoder();

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx vertSlideDownGroup = new MotorGroupEx(vertSlideRightDown);

            vertSlideDown = new SlideMotor(vertSlideDownGroup, telemetry, "vertSlideDown");
            vertSlideDown.setActionMode();
        } catch (Exception e) {
            RobotLog.e("VertSlideDown are not available");
        }

        try {
            MotorEx horizontalExtenderMotor = new MotorEx(hardwareMap, vertSlideDownName, Motor.GoBILDA.RPM_312);
            horizontalExtenderMotor.stopAndResetEncoder();

            //DON'T INVERT MOTORS AFTER HERE
            MotorGroupEx horizontalExtenderGroup = new MotorGroupEx(horizontalExtenderMotor);

            horizontalExtender = new SlideMotor(horizontalExtenderGroup, telemetry, "horizontalExtender");
        } catch (Exception e) {
            RobotLog.e("horizontalExtender are not available");
        }

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

        //if (!horizontalExtender.available) unavailableHardwares.add("horizontalExtender");

        unavailableHardwares.addAll(servoList.stream().filter(servo -> !servo.available).map(ServoDevice::getDeviceName).collect(Collectors.toList()));
    }

}
