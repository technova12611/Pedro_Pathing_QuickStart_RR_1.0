package com.roadrunner.simulation;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepV1Testing {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

//        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(16.0, -62, Math.toRadians(90)))
//                                        .strafeToLinearHeading(new Vector2d(49.2,-36),Math.PI)
//                                        .strafeTo(new Vector2d(28.5,-24.5))
//                                        .strafeTo(new Vector2d(28.5,-12.0))
//                                        .strafeTo(new Vector2d(-50,-11.0))
//                                        .strafeToLinearHeading(new Vector2d(-55,-11.50), Math.PI)
//                                        .strafeTo(new Vector2d(-50,-11.5))
//                                        .splineToLinearHeading(new Pose2d(30,-11.5, Math.PI), 0)
//                                        .splineToLinearHeading(new Pose2d(48.5,-32.0, Math.PI), 0)
//                                        .strafeTo(new Vector2d(24,-12.0))
//                                 //       .strafeTo(new Vector2d(-50,-12.0))
//                                        .strafeToLinearHeading(new Vector2d(-56,-12.0), Math.PI)
//                                        .strafeTo(new Vector2d(30,-12.0))
//                                        .strafeToLinearHeading(new Vector2d(48.5,-36.0), Math.PI)
//                                        .build());

//        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(16.0, -62, Math.toRadians(90)))
//                .strafeToLinearHeading(new Vector2d(49.2,-36),Math.PI)
//                .strafeTo(new Vector2d(28.5,-24.5))
////                .splineToLinearHeading(new Pose2d(22.5,-12.0, Math.PI), Math.PI)
////                .splineToLinearHeading(new Pose2d(-50,-11.0, Math.PI), 0)
//                .strafeTo(new Vector2d(28.5,-12.0))
//                .strafeTo(new Vector2d(-50,-11.0))
//                .strafeToLinearHeading(new Vector2d(-55,-11.50), Math.PI)
//                .strafeToLinearHeading(new Vector2d(-50,-11.5), Math.PI)
//                .strafeToLinearHeading(new Vector2d(38,-11.5), Math.PI)
//                .strafeToLinearHeading(new Vector2d(48.5,-32.0), Math.PI)
////                .splineToLinearHeading(new Pose2d(48.0,-32.0, Math.PI),0)
////                .splineToLinearHeading(new Pose2d(30,-11.5, Math.PI), 0)
////                .splineToLinearHeading(new Pose2d(48.5,-32.0, Math.PI), 0)
////                .strafeTo(new Vector2d(24,-12.0))
////                //       .strafeTo(new Vector2d(-50,-12.0))
////                .strafeToLinearHeading(new Vector2d(-56,-12.0), Math.PI)
////                .strafeTo(new Vector2d(30,-12.0))
////                .strafeToLinearHeading(new Vector2d(48.5,-36.0), Math.PI)
//                .build());

//        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(16.0, -62, Math.toRadians(90)))
//                .splineToLinearHeading(new Pose2d(46.08, -40.90, Math.toRadians(180.00)), Math.toRadians(10.00))
//                .strafeToLinearHeading(new Vector2d(29.0, -25.0), Math.toRadians(180.00))
//                .splineToConstantHeading(new Vector2d(10.51, -11.5), Math.toRadians(180.00))
//                .splineToConstantHeading(new Vector2d(-56.16, -11.5), Math.toRadians(180.00))
//                        .setReversed(true)
//                .splineTo(new Vector2d(26.00, -11.81), Math.toRadians(0.00))
//                .splineTo(new Vector2d(46.37, -30.96), Math.toRadians(-10.00))
//                .build());

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(16.0, -62, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(48.0, -40.0, Math.toRadians(180.00)), Math.toRadians(10.00))
                .strafeToLinearHeading(new Vector2d(29.0, -25.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(25.0, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-45.5, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-58.0, -11.5), Math.toRadians(180.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.00, -11.81), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(47.5, -30.0), Math.toRadians(-10.00))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.0, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-45.5, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-58.0, -11.5), Math.toRadians(180.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.00, -11.81), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(47.5, -30.0), Math.toRadians(-10.00))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35.0, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-45.5, -11.5), Math.toRadians(180.00))
                .splineToConstantHeading(new Vector2d(-58.0, -11.5), Math.toRadians(180.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.00, -11.81), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(47.5, -30.0), Math.toRadians(-10.00))
                .build());


        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

//        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(30, 30, Math.toRadians(180)))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
//                .addEntity(mySecondBot)
                .start();
    }
}