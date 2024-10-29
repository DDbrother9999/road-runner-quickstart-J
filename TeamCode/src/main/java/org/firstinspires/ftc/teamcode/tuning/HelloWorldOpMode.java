package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloWorldOpMode extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "World");
    }

    @Override
    public void loop() {

    }
}
