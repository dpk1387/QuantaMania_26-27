package com.example.meepmeeptesting.decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBlueGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)//55 speed here 70 in real robot (roadrunner)
                .build();

        double shootX = -28, shootY = -28, shootYaw = Math.toRadians(-135);
        Pose2d shootPose = new Pose2d(shootX, shootY, shootYaw);
        double newShootX = -16, newShootY = -16; //-22, -22
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(-135));

        Pose2d startPose = new Pose2d(-54, -54, Math.toRadians(-135));
        Pose2d classifierPose = new Pose2d(7.5, -60, Math.toRadians(-120));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                //indent starts here
//              preloads
                .strafeTo(new Vector2d(newShootX, newShootY))

                .waitSeconds(2) //to shoot

//                      middle row of artifacts
                .setTangent(Math.toRadians(-32))
                .splineToSplineHeading(new Pose2d(12, -42, Math.toRadians(270)), Math.toRadians(-90))
                .splineToLinearHeading(newShootPose, Math.toRadians(160))
                .waitSeconds(2)

//                        // classifier artifacts (1)
                .setTangent(Math.toRadians(-25))
                .splineToLinearHeading(classifierPose, Math.toRadians(-85))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(95)) //75, -95
                .splineToLinearHeading(newShootPose, Math.toRadians(155))
                .waitSeconds(2)

                // classifier artifacts (2)
                .setTangent(Math.toRadians(-25))
                .splineToLinearHeading(classifierPose, Math.toRadians(-85))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(95))
                .splineToLinearHeading(newShootPose, Math.toRadians(155))
                .waitSeconds(2)

                // first line of artifacts
                .setTangent(Math.toRadians(-45)) //-60
                .splineToLinearHeading(new Pose2d(-12,-44,  Math.toRadians(270)), Math.toRadians(-80)) //-100
                .splineToSplineHeading(shootPose, Math.toRadians(135)) //go into
                .waitSeconds(2)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
