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
   private float currentTicksPerMM;
   public static final float goBILDA_SWINGARM_POD = 13.262912F;
   public static final float goBILDA_4_BAR_POD = 19.894367F;

   public GoBildaPinpointDriverRR(@NotNull I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
      super(deviceClient, deviceClientIsOwned);
   }

   public final float getCurrentTicksPerMM() {
      return this.currentTicksPerMM;
   }

   public final void setCurrentTicksPerMM(float var1) {
      this.currentTicksPerMM = var1;
   }

   public void setEncoderResolution(@NotNull GoBildaPinpointDriver.GoBildaOdometryPods pods) {
      Intrinsics.checkNotNullParameter(pods, "pods");
      super.setEncoderResolution(pods);
      float var10001;
      if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
         var10001 = 13.262912F;
      } else {
         if (pods != GoBildaOdometryPods.goBILDA_4_BAR_POD) {
            throw new NotImplementedError("This odometry type not implemented in Roadrunner Pinpoint Integration");
         }

         var10001 = 19.894367F;
      }

      this.currentTicksPerMM = var10001;
   }

   public void setEncoderResolution(double ticks_per_mm) {
      super.setEncoderResolution(ticks_per_mm);
      this.currentTicksPerMM = (float)ticks_per_mm;
   }

   @NotNull
   public final Pose2d setPosition(@NotNull Pose2d pos) {
      Intrinsics.checkNotNullParameter(pos, "pos");
      this.setPosition(new Pose2D(DistanceUnit.INCH, pos.position.x, pos.position.y, AngleUnit.RADIANS, pos.heading.toDouble()));
      return pos;
   }

   @NotNull
   public final Pose2d getPositionRR() {
      return new Pose2d(this.getPosition().getX(DistanceUnit.INCH), this.getPosition().getY(DistanceUnit.INCH), this.getPosition().getHeading(AngleUnit.RADIANS));
   }

   public final void setPositionRR(@NotNull Pose2d newPose) {
      Intrinsics.checkNotNullParameter(newPose, "newPose");
      this.setPosition(new Pose2D(DistanceUnit.INCH, newPose.position.x, newPose.position.y, AngleUnit.RADIANS, newPose.heading.toDouble()));
   }

   @NotNull
   public final PoseVelocity2d getVelocityRR() {
      return new PoseVelocity2d(new Vector2d(this.getVelocity().getX(DistanceUnit.INCH), this.getVelocity().getY(DistanceUnit.INCH)), this.getVelocity().getHeading(AngleUnit.RADIANS));
   }

   public boolean initialize(@NotNull IMU.Parameters parameters) {
      Intrinsics.checkNotNullParameter(parameters, "parameters");
      return true;
   }

   public void resetYaw() {
      Pose2d curPos = this.getPositionRR();
      this.setPositionRR(new Pose2d(curPos.position, Rotation2d.Companion.fromDouble(0.0)));
   }

   @NotNull
   public YawPitchRollAngles getRobotYawPitchRollAngles() {
      return new YawPitchRollAngles(AngleUnit.RADIANS, this.getPosition().getHeading(AngleUnit.RADIANS), 0.0, 0.0, System.nanoTime());
   }

   @NotNull
   public Orientation getRobotOrientation(@NotNull AxesReference reference, @NotNull AxesOrder order, @NotNull AngleUnit angleUnit) {
      Intrinsics.checkNotNullParameter(reference, "reference");
      Intrinsics.checkNotNullParameter(order, "order");
      Intrinsics.checkNotNullParameter(angleUnit, "angleUnit");
      Orientation var4 = (new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS, 0.0F, 0.0F, (float)this.getPosition().getHeading(AngleUnit.RADIANS), System.nanoTime())).toAxesReference(reference).toAxesOrder(order).toAngleUnit(angleUnit);
      Intrinsics.checkNotNullExpressionValue(var4, "toAngleUnit(...)");
      return var4;
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
      Intrinsics.checkNotNullParameter(angleUnit, "angleUnit");
      AngularVelocity var2 = (new AngularVelocity(AngleUnit.RADIANS, 0.0F, 0.0F, (float)this.getHeadingVelocity(), System.nanoTime())).toAngleUnit(angleUnit);
      Intrinsics.checkNotNullExpressionValue(var2, "toAngleUnit(...)");
      return var2;
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
