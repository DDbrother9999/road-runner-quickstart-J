package org.firstinspires.ftc.teamcode.auto;

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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CombinedActionTeleOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.nobles.robotics.motor.HorizontalExtender;
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

    private MecanumDrive mecanumDrive;


    @Override
    public void runOpMode() {

        mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

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
                                moveInitial
                                //vertSlideUp.moveSlide(CombinedActionTeleOpMode.vertUp_targetUp, CombinedActionTeleOpMode.vertUp_maxPower),
                                //vertSlideDown.moveSlide(CombinedActionTeleOpMode.vertDown_targetUp, CombinedActionTeleOpMode.vertDown_maxPower)
                        ),
                        moveToSub,
                        new ParallelAction(
                                //clawServo1.rotateNormal(CombinedActionTeleOpMode.claw1_openDegree),
                                //clawServo2.rotateNormal(CombinedActionTeleOpMode.claw2_openDegree)
                        ),
                        push
                )
        );
    }

}
