package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36, -63, Math.toRadians(-90)))

                .strafeTo(new Vector2d(36, -9))
                .strafeTo(new Vector2d(47, -9))
                .strafeTo(new Vector2d(47, -60))
                .strafeTo(new Vector2d(47, -9))
                .strafeTo(new Vector2d(58, -9))
                .strafeTo(new Vector2d(58, -60))
                .build());

                /*
s
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(60))
                .build());

                */

                /*
                .lineToY(-34)
                .turn(Math.toRadians(-90))
                .lineToX(30)
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .turn(Math.toRadians(-45))
                .turn(Math.toRadians(45))
                .build());
                 */

                /*

                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

                 */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}