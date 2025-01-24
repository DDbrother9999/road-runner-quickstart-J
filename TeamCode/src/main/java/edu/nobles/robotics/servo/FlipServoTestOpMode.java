package edu.nobles.robotics.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.CombinedActionTeleOpMode.*;

@Autonomous(name = "FlipServoTest", group = "Autonomous")
public class FlipServoTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ServoDevice flipServo = new ServoDevice("flip0", hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            flipServo.rotateCustom(intake1Flip_initDegree, intake1Flip_oneStepTime, intake1Flip_oneStepRotation),
                            new SleepAction(5),
                            flipServo.rotateCustom(intake1Flip_flatDegree, intake1Flip_oneStepTime, intake1Flip_oneStepRotation),
                            new SleepAction(5)
                    )
            );
        }

        Actions.runBlocking(flipServo.rotateCustom(50, intake1Flip_oneStepTime, intake1Flip_oneStepRotation));
    }
}
