package org.firstinspires.ftc.teamcode;


// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// Non-RR imports
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

        import java.util.Arrays;

import edu.nobles.robotics.servo.intake1;


@Config
@Autonomous(name = "TestRR", group = "Autonomous")

public final class TestRR extends LinearOpMode {

    //Location settings
    public static int splineX = 25;
    public static int splineY = 12;
    public static int splineTan = 0;
    public static int strafeX = 0;
    public static int strafeY = 0;
    public static int splineWait = 0;
    public static int strafeWait = 0;
    public static int loops = 10;

    VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25.0),
            new AngularVelConstraint(Math.PI)
    ));
    AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-25.0, 25.0);


    public Action followTrajectoryAway(MecanumDrive drive, Pose2d beginPose) {
        Action simpleSpline = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(splineX, splineY), splineTan)
                .waitSeconds(splineWait)
                .build();
        return simpleSpline;
    }

    public Action followTrajectoryReturn(MecanumDrive drive, Pose2d beginPose) {
        Action simpleStrafe = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(strafeX, strafeY))
                .waitSeconds(strafeWait)
                .build();
        return simpleStrafe;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ServoEx flip0 = new SimpleServo(
                hardwareMap, "flip0", 0, 300,
                AngleUnit.DEGREES
        );
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //start pose
        Pose2d beginPose = new Pose2d(0, 0, 0);
        intake1 testIntake1 = new intake1(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Angle", flip0.getAngle());

        int i = 0;
        while (i < loops) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    followTrajectoryAway(drive, beginPose),
                                    testIntake1.flip300()
                            ),
                            new ParallelAction(
                                    followTrajectoryReturn(drive, beginPose),
                                    testIntake1.flip0()
                            )
                    )
            );
            i++;
        }
    }
}

