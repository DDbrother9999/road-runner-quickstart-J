package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "FinalAuto", group = "Autonomous")
public class FinalAuto extends LinearOpMode {
    public static double startX= 36;
    public static double startY= -62.5;
    public static double startHeading= -90;

    public static double firstX= -90;
    public static double targetY= -90;

    //start pose
    private Pose2d beginPose = new Pose2d(36, -62.5, Math.toRadians(-90));

    private MecanumDrive mecanumDrive;


    @Override
    public void runOpMode() {

        mecanumDrive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder pushTraj = mecanumDrive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(36, -9))
                .strafeTo(new Vector2d(47, -9))
                .strafeTo(new Vector2d(47, -60))
                .strafeTo(new Vector2d(47, -9))
                .strafeTo(new Vector2d(58, -9))
                .strafeTo(new Vector2d(58, -60));

        Action push = pushTraj.build();

        // actions that need to happen on init; for instance, a claw tightening.
        // INSERT HERE

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
            push
        );
    }

}
