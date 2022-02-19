package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.5, 61, Math.toRadians(90)))
                                .strafeTo(new Vector2d(-11.8, 47.4))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(9.5, 61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, 61), Math.toRadians(0))
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(9.5, 61), Math.toRadians(0))
                                .strafeTo(new Vector2d(-11.8, 47.4))
                                .turn(Math.toRadians(90))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(9.5, 61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, 61), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.5, -61, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-11.8, -47.4))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(9.5, -61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -61), Math.toRadians(0))

                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(9.5, -61), Math.toRadians(0))
                                .strafeTo(new Vector2d(-11.8, -47.4))
                                .turn(Math.toRadians(-90))
                                .waitSeconds(2)
                                .splineTo(new Vector2d(9.5, -61), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -61), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBot)
                .addEntity(redBot)
                .start();
    }

}