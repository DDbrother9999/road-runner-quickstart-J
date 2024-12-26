package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name = "TestRR", group = "Autonomous")

public final class TestRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            int i=0;
            while(i<5) {
                Action simpleSpline = drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 22), 0)
                        .strafeTo(new Vector2d(0, 0))
                        .build();
                Actions.runBlocking(
                        simpleSpline);
                Actions.SleepAction();

                i++;
            }
        }
        else {
            throw new RuntimeException();
        }
    }
}

