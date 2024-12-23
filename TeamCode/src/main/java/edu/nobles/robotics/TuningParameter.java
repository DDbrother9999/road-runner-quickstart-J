package edu.nobles.robotics;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class TuningParameter extends MecanumDrive.Params {
    public static TuningParameter current  = new TuningParameter();

    static {
        setUpFirstTestingRobot();
        //setUpRealRobot();
    }

    public DcMotorSimple.Direction leftFrontDirection = FORWARD;
    public DcMotorSimple.Direction leftBackDirection = FORWARD;
    public DcMotorSimple.Direction rightFrontDirection = FORWARD;
    public DcMotorSimple.Direction rightBackDirection = FORWARD;

    private static void setUpFirstTestingRobot(){
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        current.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        current.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        // drive model parameters
        current.inPerTick = 0.0231696995344901;
        current.lateralInPerTick = 0.0228513095084505;
        current.trackWidthTicks = 922.6943228960521;

        // feedforward parameters (in tick units)
        current.kS = 0.7463453105505442;
        current.kV = 0.004149425258238021;
        current.kA = 0.00001;

        // path profile parameters (in inches)
        current.maxWheelVel = 10;
        current.minProfileAccel = -6;
        current.maxProfileAccel = 10;

        // turn profile parameters (in radians)
        current.maxAngVel = Math.PI; // shared with path
        current.maxAngAccel = Math.PI;

        // path controller gains
        current.axialGain = 4.0;
        current.lateralGain = 3.0;
        current.headingGain = 2.0; // shared with turn

        current.axialVelGain = 0.0;
        current.lateralVelGain = 0.0;
        current.headingVelGain = 0.1; // shared with turn

        current.leftFrontDirection = REVERSE;
        current.leftBackDirection= REVERSE;

    }

    private static void setUpRealRobot(){
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        current.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        current.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        // drive model parameters
        current.inPerTick = 0.0231696995344901;
        current.lateralInPerTick = 0.0228513095084505;
        current.trackWidthTicks = 922.6943228960521;

        // feedforward parameters (in tick units)
        current.kS = 0.7463453105505442;
        current.kV = 0.004149425258238021;
        current.kA = 0.00001;

        // path profile parameters (in inches)
        current.maxWheelVel = 10;
        current.minProfileAccel = -6;
        current.maxProfileAccel = 10;

        // turn profile parameters (in radians)
        current.maxAngVel = Math.PI; // shared with path
        current.maxAngAccel = Math.PI;

        // path controller gains
        current.axialGain = 4.0;
        current.lateralGain = 3.0;
        current.headingGain = 2.0; // shared with turn

        current.axialVelGain = 0.0;
        current.lateralVelGain = 0.0;
        current.headingVelGain = 0.1; // shared with turn

        current.leftFrontDirection = REVERSE;
        current.leftBackDirection= REVERSE;
    }

}
