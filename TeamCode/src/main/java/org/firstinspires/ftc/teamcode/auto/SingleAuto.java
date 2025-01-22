package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CombinedActionTeleOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.motor.MotorGroupEx;
import edu.nobles.robotics.motor.SlideMotor;
import edu.nobles.robotics.servo.ServoDevice;
import edu.nobles.robotics.servo.intake1;


@Config
@Autonomous(name = "SingleAuto", group = "Autonomous")
public class SingleAuto extends LinearOpMode {
    public static int splineX = 25;
    public static int splineY = 12;
    public static int splineTan = 0;
    public static int strafeX = 0;
    public static int strafeY = 0;
    public static int splineWait = 0;
    public static int strafeWait = 0;
    public static int loops = 10;

    private boolean flipFlat = false;
    private boolean clawOpen = true;
    private boolean contRotateStoped = true;

    //start pose
    private Pose2d beginPose = new Pose2d(0, -62.5, Math.toRadians(90));
    private intake1 testIntake1 = new intake1(hardwareMap);
    private MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

    private MecanumDrive mecanumDrive;
    private ServoDevice flipServo;
    private ServoDevice clawServo1;
    private ServoDevice clawServo2;
    private CRServo contRotateServo;
    private ServoDevice servoArmSpinner;
    private SlideMotor vertSlideUp;
    private SlideMotor vertSlideDown;


    @Override
    public void runOpMode() {
        initHardeware();

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
                .strafeTo(new Vector2d(56, -60))
                .strafeTo(new Vector2d(56, 0))
                .strafeTo(new Vector2d(56, 0));

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
                                vertSlideUp.moveSlide(CombinedActionTeleOpMode.vertUp_targetUp, CombinedActionTeleOpMode.vertUp_maxPower),
                                vertSlideDown.moveSlide(CombinedActionTeleOpMode.vertDown_targetUp, CombinedActionTeleOpMode.vertDown_maxPower)
                        ),
                        moveToSub,
                        new ParallelAction(
                                clawServo1.rotateNormal(CombinedActionTeleOpMode.claw1_openDegree),
                                clawServo2.rotateNormal(CombinedActionTeleOpMode.claw2_openDegree)
                        ),
                        push
                )
        );
    }

    private void initHardeware() {
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

        //NO GAMEPADS

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

}
