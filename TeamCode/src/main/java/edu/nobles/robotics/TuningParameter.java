package edu.nobles.robotics;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class TuningParameter extends MecanumDrive.Params {

    public static TuningParameter current = setUpRealRobot();
    // public static TuningParameter current = setUpFirstTestingRobot();

    public Class<?> DRIVE_CLASS;

    public boolean usePinpointDevice;

    public String imuName;

    public DcMotorSimple.Direction leftFrontDirection = FORWARD;
    public DcMotorSimple.Direction leftBackDirection = FORWARD;
    public DcMotorSimple.Direction rightFrontDirection = FORWARD;
    public DcMotorSimple.Direction rightBackDirection = FORWARD;

    public PinpointParams pinpointParams;

    private static TuningParameter setUpRealRobot() {
        TuningParameter param = new TuningParameter();

        param.DRIVE_CLASS = MecanumDrive.class;

        param.usePinpointDevice = true;
        param.pinpointParams = new PinpointParams();

        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        param.imuName = "pinpoint";
        param.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        param.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        // 0.0019789560019118
        param.inPerTick = 1.0d / 25.4d / param.pinpointParams.encoderResolution;
        param.lateralInPerTick = 0.001623458773196117;
        param.trackWidthTicks = 7479.9620110751885;

        // feedforward parameters (in tick units)
        param.kS = 0.8413297455354614;
        param.kV = 0.00036203672651163404;
        param.kA = 0.0001;

        // path profile parameters (in inches)
        param.maxWheelVel = 10;
        param.minProfileAccel = -6;
        param.maxProfileAccel = 10;

        // turn profile parameters (in radians)
        param.maxAngVel = Math.PI; // shared with path
        param.maxAngAccel = Math.PI;

        // path controller gains
        param.axialGain = 3.0;
        param.lateralGain = 1.0;
        param.headingGain = 2.0; // shared with turn

        param.axialVelGain = 0.0;
        param.lateralVelGain = 0.0;
        param.headingVelGain = 0.0; // shared with turn

        param.leftFrontDirection = REVERSE;
        param.leftBackDirection = REVERSE;

        return param;
    }

    private static TuningParameter setUpFirstTestingRobot() {
        TuningParameter param = new TuningParameter();

        param.DRIVE_CLASS = MecanumDrive.class;

        param.usePinpointDevice = false;
        param.pinpointParams = null;

        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        param.imuName = "imu";
        param.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        param.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        // drive model parameters
        param.inPerTick = 0.0231696995344901;
        param.lateralInPerTick = 0.0228513095084505;
        param.trackWidthTicks = 922.6943228960521;

        // feedforward parameters (in tick units)
        param.kS = 0.7463453105505442;
        param.kV = 0.004149425258238021;
        param.kA = 0.00001;

        // path profile parameters (in inches)
        param.maxWheelVel = 10;
        param.minProfileAccel = -6;
        param.maxProfileAccel = 10;

        // turn profile parameters (in radians)
        param.maxAngVel = Math.PI; // shared with path
        param.maxAngAccel = Math.PI;

        // path controller gains
        param.axialGain = 4.0;
        param.lateralGain = 3.0;
        param.headingGain = 2.0; // shared with turn

        param.axialVelGain = 0.0;
        param.lateralVelGain = 0.0;
        param.headingVelGain = 0.1; // shared with turn

        param.leftFrontDirection = REVERSE;
        param.leftBackDirection = REVERSE;

        return param;
    }
}
