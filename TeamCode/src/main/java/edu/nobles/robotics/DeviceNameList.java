package edu.nobles.robotics;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DeviceNameList {
    // Servos
    public static String clawLeftName = "servoExp0";
    public static String clawRightName = "servoExp1";
    public static String intake1SlideRetractName = "servoExp3";
    public static String intake1SlideExtendName = "servoExp5";
    public static String intake1FlipName = "servo3";
    public static String intake1spinName = "servo5";

    // Motors
    public static String frontLeftName = "frontLeft";
    public static String backLeftName = "backLeft";
    public static String backRightName = "backRight";
    public static String frontRightName = "frontRight";
    public static String vertSlideRightUpName = "vertSlideRightUp";
    public static String vertSlideRightDownName = "vertSlideRightDown";
    public static String vertSlideLeftUpName = "vertSlideLeftUp";
    public static String vertSlideLeftDownName = "vertSlideLeftDown";
}
