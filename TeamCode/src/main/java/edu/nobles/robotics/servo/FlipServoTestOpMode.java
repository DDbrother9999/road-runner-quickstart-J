package edu.nobles.robotics.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "FlipServoTest", group = "Autonomous")
public class FlipServoTestOpMode  extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ServoDevice flipServo = new ServoDevice("flip0", hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            flipServo.rotate(ServoDevice.flip0_InitDegree),
                            new SleepAction(5),
                            flipServo.rotate(ServoDevice.flip0_FlatDegree),
                            new SleepAction(5)
                    )
            );
        }

        Actions.runBlocking(flipServo.rotate(50));
    }
}
