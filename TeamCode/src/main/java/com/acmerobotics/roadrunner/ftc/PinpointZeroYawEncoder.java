package com.acmerobotics.roadrunner.ftc;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.jetbrains.annotations.NotNull;

import kotlin.jvm.internal.Intrinsics;

public final class PinpointZeroYawEncoder {
   @NotNull
   private final GoBildaPinpointDriverRR pinpoint;
   private final boolean usePerpendicular;
   @NotNull
   private final DcMotor anyDummyMotor;
   @NotNull
   private DcMotorSimple.Direction direction;

   public PinpointZeroYawEncoder(@NotNull GoBildaPinpointDriverRR pinpoint, boolean usePerpendicular, @NotNull DcMotor anyDummyMotor) {
      super();
      this.pinpoint = pinpoint;
      this.usePerpendicular = usePerpendicular;
      this.anyDummyMotor = anyDummyMotor;
      Log.println(Log.INFO, "PinpointEncoder", "init: Initializing pinpoint encoder in tuning mode");
      Log.println(Log.INFO, "PinpointEncoder", "init: Old yaw scalar = " + this.pinpoint.getYawScalar());
      Log.println(Log.WARN, "PinpointEncoder", "init: Setting Pinpoint yaw scalar to 0. Perform power cycle to reset");
      Object[] var4 = new Object[]{this.pinpoint.getYawScalar()};
      RobotLog.addGlobalWarningMessage("Disabling Pinpoint IMU. Perform a power cycle (turn the robot off and back on again) to reset it before running Feedback Tuner, LocalizationTest, or an auto (Angular Scalar now 0, previously %f)", var4);
      this.pinpoint.setYawScalar(0.0);
      this.pinpoint.resetPosAndIMU();
      this.direction = DcMotorSimple.Direction.FORWARD;
   }

   @NotNull
   public DcMotorSimple.Direction getDirection() {
      return this.direction;
   }

   public void setDirection(@NotNull DcMotorSimple.Direction var1) {
      Intrinsics.checkNotNullParameter(var1, "<set-?>");
      this.direction = var1;
   }

   @NotNull
   public PositionVelocityPair getPositionAndVelocity() {
      this.pinpoint.update();
      double pos = 0.0;
      double vel = 0.0;
      if (this.usePerpendicular) {
         pos = this.pinpoint.getPositionRR().position.y;
         vel = this.pinpoint.getVelocityRR().linearVel.y;
      } else {
         pos = this.pinpoint.getPositionRR().position.x;
         vel = this.pinpoint.getVelocityRR().linearVel.x;
      }

      return new PositionVelocityPair((int)pos, (int)vel, (int)pos, (int)vel);
   }

   @NotNull
   public DcMotorController getController() {
      DcMotorController var1 = this.anyDummyMotor.getController();
      Intrinsics.checkNotNullExpressionValue(var1, "getController(...)");
      return var1;
   }
}
