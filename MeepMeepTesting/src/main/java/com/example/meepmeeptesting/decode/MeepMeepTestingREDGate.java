package com.example.meepmeeptesting.decode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingREDGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)//55 speed here 70 in real robot (roadrunner)
                .build();
        double shootX = -28, shootY = 28;
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(135));

        //longer shot
        //shorter movement to classifier
        double newShootX = -23, newShootY = 23; // -22, 22
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(135));

        Pose2d startPose = new Pose2d(-54, 54, Math.toRadians(135));
        Pose2d classifierPose = new Pose2d(7.5, 64, Math.toRadians(120));

        //not necessary
        Pose2d readyPose = new Pose2d(4, -30, Math.toRadians(-45));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                        //SHOOT PRELOADS
                        .strafeTo(new Vector2d(newShootX, newShootY))
                        .waitSeconds(2)

                        //.turn(-Math.toRadians(100))
                        //.strafeTo(new Vector2d(-20, 40))

                        // MIDDLE ROW OF ARTIFACTS
                        .setTangent(Math.toRadians(-3))
//                .splineToSplineHeading(new Pose2d(12.4, 43, Math.toRadians(92)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(7.5, 29, Math.toRadians(85)), Math.toRadians(50))
                        .splineToSplineHeading(new Pose2d(11.5, 46, Math.toRadians(95)), Math.toRadians(90))


                        //.setTangent(Math.toRadians(32)) //15
                        //go to intake balls
                        //.splineToSplineHeading(new Pose2d(12, 42, Math.toRadians(90)), Math.toRadians(90)) //_, _,_, 95
                        //go to shoot

//                .setTangent(Math.toRadians(-100))
                        .splineToLinearHeading(new Pose2d(0, 32, Math.toRadians(120)), Math.toRadians(-135)) //200
                        .splineToSplineHeading(newShootPose, Math.toRadians(-180)) //200
                        .waitSeconds(2)

                        // classifier artifacts (1)
                        .setTangent(Math.toRadians(5))

                        //.setTangent(Math.toRadians(25)) //15
                        .splineToSplineHeading(new Pose2d(0, 30, Math.toRadians(135)), Math.toRadians(55))
                        .splineToLinearHeading(classifierPose, Math.toRadians(95)) //85 //95 //go into
                        .waitSeconds(1.5)

                        .setTangent(Math.toRadians(-90)) //-95 //-90
                        .splineToSplineHeading(new Pose2d(0, 30, Math.toRadians(135)), Math.toRadians(-132))
                        .splineToLinearHeading(newShootPose, Math.toRadians(-175)) //-155 //200 //go into
                        .waitSeconds(2)

                        // classifier artifacts (2)
                        .setTangent(Math.toRadians(5))

                        //.setTangent(Math.toRadians(25)) //15
                        .splineToSplineHeading(new Pose2d(0, 30, Math.toRadians(125)), Math.toRadians(55))
                        .splineToLinearHeading(classifierPose, Math.toRadians(95)) //85 //95 //go into
                        .waitSeconds(1.5)

                        .setTangent(Math.toRadians(-90)) //-95 //-90
                        .splineToSplineHeading(new Pose2d(0, 30, Math.toRadians(135)), Math.toRadians(-132))
                        .splineToLinearHeading(newShootPose, Math.toRadians(-175)) //-155 //200 //go into
                        .waitSeconds(2)

                        // first line of artifacts

                        .setTangent(Math.toRadians(0)) //60
                        //go into
                        //.splineToLinearHeading(new Pose2d(-12, 44,  Math.toRadians(90)), Math.toRadians(100))
                        .splineToSplineHeading(new Pose2d(-12, 36,  Math.toRadians(85)), Math.toRadians(85))
                        .splineToLinearHeading(new Pose2d(-13.5, 48, Math.toRadians(100)), Math.toRadians(115))
                        //go back to shooting
                        .splineToSplineHeading(shootPose, Math.toRadians(225)) //-135 //go into
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