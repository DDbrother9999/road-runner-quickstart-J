package com.acmerobotics.roadrunner.ftc;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import kotlin.Metadata;
import kotlin.NotImplementedError;
import kotlin.jvm.internal.DefaultConstructorMarker;
import kotlin.jvm.internal.Intrinsics;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.jetbrains.annotations.NotNull;

@I2cDeviceType
@DeviceProperties(
   xmlTag = "goBILDAPinpointRR",
   name = "goBILDA® Pinpoint Odometry Computer Roadrunner Driver",
   description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
public final class GoBildaPinpointDriverRR extends GoBildaPinpointDriver implements IMU {
   @NotNull
   public static final Companion Companion = new Companion((DefaultConstructorMarker)null);

   public GoBildaPinpointDriverRR(@NotNull I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
      super(deviceClient, deviceClientIsOwned);
   }

   @NotNull
   public final Pose2d setPosition(@NotNull Pose2d pos) {
      this.setPosition(new Pose2D(DistanceUnit.INCH, pos.position.x, pos.position.y, AngleUnit.RADIANS, pos.heading.toDouble()));
      return pos;
   }

   public boolean initialize(@NotNull IMU.Parameters parameters) {
      return true;
   }

   public void resetYaw() {
   }

   @NotNull
   public YawPitchRollAngles getRobotYawPitchRollAngles() {
      return new YawPitchRollAngles(AngleUnit.RADIANS, this.getPosition().getHeading(AngleUnit.RADIANS), 0.0, 0.0, System.nanoTime());
   }

   @NotNull
   public Orientation getRobotOrientation(@NotNull AxesReference reference, @NotNull AxesOrder order, @NotNull AngleUnit angleUnit) {
      return (new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, 0.0F, 0.0F,
              (float) this.getPosition().getHeading(AngleUnit.RADIANS), System.nanoTime()))
              .toAxesReference(reference).toAxesOrder(order).toAngleUnit(angleUnit);
   }

   public static Quaternion eulerToQuaternion(double yaw) {
      double qx = Math.cos(yaw / (double)2) - Math.sin(yaw / (double)2);
      double qy = Math.cos(yaw / (double)2) + Math.sin(yaw / (double)2);
      double qz = Math.sin(yaw / (double)2) - Math.cos(yaw / (double)2);
      double qw = Math.cos(yaw / (double)2) + Math.sin(yaw / (double)2);
      return new Quaternion((float)qw, (float)qx, (float)qy, (float)qz, System.nanoTime());
   }

   @NotNull
   public Quaternion getRobotOrientationAsQuaternion() {
      return eulerToQuaternion(this.getPosition().getHeading(AngleUnit.RADIANS));
   }

   @NotNull
   public AngularVelocity getRobotAngularVelocity(@NotNull AngleUnit angleUnit) {
      return (new AngularVelocity(AngleUnit.RADIANS, 0.0F, 0.0F,
              (float) this.getHeadingVelocity(), System.nanoTime())).toAngleUnit(angleUnit);
   }

   public static final class Companion {
      private Companion() {
      }

      // $FF: synthetic method
      public Companion(DefaultConstructorMarker $constructor_marker) {
         this();
      }
   }
}
