package com.roadrunner.simulation.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16,17)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16.0, -62, Math.toRadians(90)))
                                        .setTangent(0)
                                        .splineTo(new Vector2d(49.2,-36),Math.toRadians(90))
                                        .strafeTo(new Vector2d(28.5,-24.5))
                                        .strafeTo(new Vector2d(28.5,-12.0))
//                                        .strafeTo(new Vector2d(-52,-12.0))
//                                        .strafeTo(new Vector2d(-58,-12.0))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(30,-12.0), Math.toRadians(0))
//                                        .setTangent(0)
//                                        .strafeTo(new Vector2d(48,-36.0))
//                                        .setReversed(false)
//                                        .strafeTo(new Vector2d(24,-12.0))
//                                        .strafeTo(new Vector2d(-50,-12.0))
//                                        .strafeTo(new Vector2d(-53,-13.0))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(30,-12.0), Math.toRadians(0))
//                                        .strafeTo(new Vector2d(49,-36.0))
                                        .build()
                );

        // this is the BLUE left path
        //----------------------------
//        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
//                .setDimensions(16,18)
//                .setColorScheme(new ColorSchemeBlueDark())
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 16)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(16.0, 62, Math.toRadians(-90)))
//                                .setTangent(0)
//                                .splineTo(new Vector2d(49.2,36),Math.toRadians(-90))
//                                .strafeTo(new Vector2d(28.5,24.5))
//                                .strafeTo(new Vector2d(28.5,12.0))
//                                .strafeTo(new Vector2d(-52,12.0))
//                                .strafeTo(new Vector2d(-58,12.0))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(30,12.0), Math.toRadians(0))
//                                .strafeTo(new Vector2d(48,36.0))
//                                .setReversed(false)
//                                .strafeTo(new Vector2d(30,12.0))
//                                .strafeTo(new Vector2d(-52,12.0))
//                                .strafeTo(new Vector2d(-58,12.0))
//                                .setReversed(true)
//                                .splineTo(new Vector2d(30,12.0), Math.toRadians(0))
//                                .strafeTo(new Vector2d(48,36.0))
//                                .build()
//                );

                RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16,17)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40.0, -62, Math.toRadians(90)))
                                .splineTo(new Vector2d(-35.0,-36), Math.toRadians(30))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-40,-48, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(false)
                                .strafeTo(new Vector2d(-53.0,-25))
                                .strafeTo(new Vector2d(-48.0,-12))
//
//                                .lineTo(new Vector2d(-34, -37))
//                                .setReversed(true)
//                                .lineTo(new Vector2d(-36, -40))
//                                .turn(Math.toRadians(90))
//                                .lineTo(new Vector2d(-55, -38))
//
//
//                                .lineTo(new Vector2d(-48, -45))
//                                .strafeTo(new Vector2d(-36, -45))
//                                .lineTo(new Vector2d(-36, -12))
//                                .turn(Math.toRadians(90))
//                                .lineTo(new Vector2d(-54, -12))

                                .setReversed(true)
                                .lineTo(new Vector2d(40, -12))
                                .strafeTo(new Vector2d(49, -29))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(mySecondBot)
                .start();
    }
}