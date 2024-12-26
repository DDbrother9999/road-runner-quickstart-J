package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name = "TestRR", group = "Autonomous")

public final class TestRR extends LinearOpMode {

    //Location settings
    public static int splineX = 30;
    public static int splineY = 18;
    public static int splineTan = 0;
    public static int strafeX = 0;
    public static int strafeY = 0;
    public static int splineWait = 0;
    public static int strafeWait = 0;
    public static int loops = 10;
    public Action followTrajectory(MecanumDrive drive, Pose2d beginPose) {
        Action simpleSpline = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(splineX, splineY), splineTan)
                .waitSeconds(splineWait)
                .strafeTo(new Vector2d(strafeX, strafeY))
                .waitSeconds(strafeWait)
                .build();
        return simpleSpline;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        
        //start pose
        Pose2d beginPose = new Pose2d(0, 0, 0);

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            int i=0;
            while(i<loops) {
                Actions.runBlocking(
                        new SequentialAction(
                            followTrajectory(drive, beginPose),
                            new SleepAction(2)
                        )
                );
                i++;
            }
    }
}

