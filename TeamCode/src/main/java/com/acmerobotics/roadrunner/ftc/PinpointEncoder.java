// PinpointEncoder.java
package com.acmerobotics.roadrunner.ftc;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.RobotLog;

import kotlin.Metadata;
import kotlin.jvm.internal.Intrinsics;
import org.jetbrains.annotations.NotNull;

public class PinpointEncoder {
   @NotNull
   private final GoBildaPinpointDriverRR pinpoint;
   private final boolean usePerpendicular;
   @NotNull
   private final DcMotor anyDummyMotor;
   @NotNull
   private DcMotorSimple.Direction direction;

   public PinpointEncoder(@NotNull GoBildaPinpointDriverRR pinpoint, boolean usePerpendicular, @NotNull DcMotor anyDummyMotor) {
      super();
      this.pinpoint = pinpoint;
      this.usePerpendicular = usePerpendicular;
      this.anyDummyMotor = anyDummyMotor;
      this.direction = Direction.FORWARD;
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
      Pose2d absPose = this.pinpoint.getPositionRR();
      PoseVelocity2d absVel = this.pinpoint.getVelocityRR();
      Vector2d rotatedPosVector = absPose.position;
      Vector2d rotatedVelVector = absVel.linearVel;
      int pos = 0;
      int vel = 0;
      if (this.usePerpendicular) {
         pos = (int)rotatedPosVector.y; // QUESTION: double
         vel = (int)rotatedVelVector.y; // QUESTION: double
      } else {
         pos = (int)rotatedPosVector.x; // QUESTION: double
         vel = (int)rotatedVelVector.x; // QUESTION: double
      }

      return new PositionVelocityPair(pos, vel, pos, vel);
   }

   @NotNull
   public DcMotorController getController() {
      DcMotorController var1 = this.anyDummyMotor.getController();
      Intrinsics.checkNotNullExpressionValue(var1, "getController(...)");
      return var1;
   }
}

