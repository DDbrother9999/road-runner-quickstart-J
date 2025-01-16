package edu.nobles.robotics.sensors;

import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DistanceSensor{

    private SensorRevTOFDistance distanceSensor;

    public DistanceSensor(SensorRevTOFDistance distanceSensor){
        this.distanceSensor=distanceSensor;
    }

}
