package edu.nobles.robotics;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class PinpointParams {
        /*
        Set this to the name that your Pinpoint is configured as in your hardware config.
         */
        public String pinpointDeviceName = "pinpoint";

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of the center is a negative number. The Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is: forward of the center is a positive number,
        backwards is a negative number.
         */
        // These units are mm
        public double xOffset = -90d;
        public double yOffset = -165d;

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, input the number of ticks per millimeter for that pod.

        RR LOCALIZER NOTE: this is ticks per MILLIMETER, NOT inches per tick.
        This value should be more than one; the value for the Gobilda 4 Bar Pod is approximately 20.
        To get this value from inPerTick, first convert the value to millimeters (multiply by 25.4)
        and then take its inverse (one over the value)
         */
//        public static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
//        public static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod
        public double encoderResolution = 19.89436789f;

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    }
