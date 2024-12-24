// PinpointRawPassthroughEncoder.java
package com.acmerobotics.roadrunner.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import kotlin.Metadata;
import kotlin.jvm.internal.Intrinsics;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.NotNull;

public final class PinpointRawPassthroughEncoder {
   @NotNull
   private final GoBildaPinpointDriverRR pinpoint;
   private final boolean usePerpendicular;
   private final boolean reversed;
   @NotNull
   private final DcMotor anyDummyMotor;
   private final boolean autoUpdate;
   @NotNull
   private DcMotorSimple.Direction direction;
   private double lastPos;
   private long lastTime;

   public PinpointRawPassthroughEncoder(@NotNull GoBildaPinpointDriverRR pinpoint, boolean usePerpendicular, boolean reversed, @NotNull DcMotor anyDummyMotor, boolean autoUpdate) {
      super();
      this.pinpoint = pinpoint;
      this.usePerpendicular = usePerpendicular;
      this.reversed = reversed;
      this.anyDummyMotor = anyDummyMotor;
      this.autoUpdate = autoUpdate;
      this.direction = Direction.FORWARD;
      this.lastTime = System.nanoTime();
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
      if (this.autoUpdate) {
         this.pinpoint.update();
      }

      double pos = 0.0;
      double vel = 0.0;
      if (this.usePerpendicular) {
         pos = DistanceUnit.INCH.fromMm((double)((float)this.pinpoint.getEncoderY() / this.pinpoint.getCurrentTicksPerMM()));
      } else {
         pos = DistanceUnit.INCH.fromMm((double)((float)this.pinpoint.getEncoderX() / this.pinpoint.getCurrentTicksPerMM()));
      }

      if (this.reversed) {
         pos *= (double)-1;
      }

      if (this.lastPos == 0.0) {
         this.lastPos = pos;
         vel = 0.0;
      } else {
         long currentTime = System.nanoTime();
         double timeDiffSec = (double)(currentTime - this.lastTime) * 1.0E-9;
         vel = (pos - this.lastPos) / timeDiffSec;
         this.lastPos = pos;
         this.lastTime = currentTime;
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
