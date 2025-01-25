package edu.nobles.robotics.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

@TeleOp
public class TestSimpleServo extends LinearOpMode {
    public static String servoName = "servoArmFlip";


    public static int slowdown = 2;
    public static int aTurnTo = 150;
    public static double turnBy = 5;
    public static int minAng = 0;
    public static int maxAng = 300;


    @Override
    public void runOpMode() throws InterruptedException {

        ServoEx flip0 = new SimpleServo(
                hardwareMap, servoName, minAng, maxAng,
                AngleUnit.DEGREES
        );
        FtcDashboard dashboard = FtcDashboard.getInstance();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double triggerSpin = gamepad1.left_stick_y;
            flip0.rotateByAngle(triggerSpin/slowdown);

            if(gamepad1.a){
                flip0.turnToAngle(aTurnTo, AngleUnit.DEGREES);
            }
            else if (gamepad1.b) {
                flip0.setPosition(0);
            }
            else if (gamepad1.y) {
                flip0.setPosition(1);
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle", flip0.getAngle());
            telemetry.addData("Angle", flip0.getAngle());
            packet.put("Position", flip0.getPosition());
            telemetry.addData("Position", flip0.getPosition());
            packet.put("left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            packet.put("a", gamepad1.a);
            packet.put("b", gamepad1.a);
            packet.put("y", gamepad1.a);

            packet.put("status", "alive");
            dashboard.sendTelemetryPacket(packet);



        }
    }
}