package edu.nobles.robotics.parameters;

import com.acmerobotics.dashboard.config.Config;

@Config

public class ParameterManager {
    public static long fullExtend  = 10;

    public static double maxVertUpPower = 0.25;
    public static double maxVertDownPower = maxVertUpPower;

    public static int targetVertUpExtend = 2000;
    public static int targetVertDownExtend = targetVertUpExtend;
    public static int targetVertUpRetract = 0;
    public static int targetVertDownRetract = targetVertUpRetract;

    public static double fullArmExtendTime = 10;
    public static double fullArmRetractTime = 10;


}
