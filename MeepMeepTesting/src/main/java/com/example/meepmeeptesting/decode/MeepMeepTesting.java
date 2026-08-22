package com.example.meepmeeptesting.decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)//55 speed here 70 in real robot (roadrunner)
                .build();
        double shootX = -30, shootY = 30;
        Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(135));
        Pose2d startPose = new Pose2d(-48, 48, Math.toRadians(135));
        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .strafeTo(new Vector2d(shootX, shootY))

                .waitSeconds(3) //to shoot
                //.turn(-Math.toRadians(100))
                //.strafeTo(new Vector2d(-20, 40))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-24, 38,  Math.toRadians(45)), Math.toRadians(45)) //go into
                .splineToLinearHeading(new Pose2d(-6, 58,  Math.toRadians(90)), Math.toRadians(90)) //go into
                //.splineToLinearHeading(new Pose2d(-8, 54,  Math.toRadians(180)), Math.toRadians(180)) //go into
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(shootPos, Math.toRadians(225)) //go into
                .waitSeconds(3)

                //.turn(-Math.toRadians(120))
                //.strafeTo(new Vector2d(0, 38))
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(new Pose2d(4, 38,  Math.toRadians(45)), Math.toRadians(15)) //go into
                .splineToLinearHeading(new Pose2d(8, 60,  Math.toRadians(115)), Math.toRadians(95)) //go into
                .setTangent(Math.toRadians(-70))
                .splineToLinearHeading(shootPos, Math.toRadians(200)) //go into
                .waitSeconds(3)

                //.turn(-Math.toRadians(125))
                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(26, 35,  Math.toRadians(45)), Math.toRadians(0)) //go into
                .splineToLinearHeading(new Pose2d(33, 58,  Math.toRadians(135)), Math.toRadians(135)) //go into
                .setTangent(Math.toRadians(200))
                .splineToLinearHeading(shootPos, Math.toRadians(200)) //go into
//                .splineToLinearHeading(new Pose2d(-11, 28,  Math.toRadians(90)), Math.toRadians(45)) //go into
//                //.splineTo(new Vector2d(-11, 24), Math.toRadians(90))
//                .strafeTo(new Vector2d(-11, 52))
//                .strafeTo(new Vector2d(-6, 46))
//                .strafeTo(new Vector2d(-6, 52))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
//                .waitSeconds(3) //to shoot
//
//                .setTangent(Math.toRadians(10))
//                .splineToLinearHeading(new Pose2d(11, 28,  Math.toRadians(90)), Math.toRadians(20))
//                .strafeTo(new Vector2d(11, 56))
//                .strafeTo(new Vector2d(6, 48))
//                .strafeTo(new Vector2d(-6, 52))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
//                .waitSeconds(3) //to shoot
//
//                .setTangent(Math.toRadians(10))
//                .splineToLinearHeading(new Pose2d(33, 28,  Math.toRadians(90)), Math.toRadians(15))
//                .strafeTo(new Vector2d(33, 56))
//                .setTangent(Math.toRadians(225))
//                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
                .waitSeconds(3) //to shoot
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}