package edu.nobles.robotics.parameters;

import com.acmerobotics.dashboard.config.Config;

@Config

public class ParameterManager {
    public static int highChamberExtendUpPos = 3000;
    public static int highChamberExtendDownPos = highChamberExtendUpPos *312/435;
    public static int highChamberRetractUpPos = 2500;
    public static int highChamberRetractDownPos = highChamberExtendDownPos *312/435;

}
