package edu.nobles.robotics;

import com.acmerobotics.dashboard.config.Config;

//Key: if not specified, is on control hub

@Config
public class DeviceNameList {
    // Servos
    public static String clawName = "servoExp0";

    public static String intake1FlipName = "servo3";
    public static String intake1spinName = "servo5";

    // Motors
    public static String intake1SlideExtendName = "motor3"; //gobilda 435

    public static String frontLeftName = "motor1";
    public static String backLeftName = "motor2";
    public static String backRightName = "motorExp1";
    public static String frontRightName = "motorExp3";
    public static String vertSlideUpName = "motorExp0"; //gobilda 435
    public static String vertSlideDownName = "motor0"; //gobilda 312
}
