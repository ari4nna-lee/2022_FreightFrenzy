package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -62, Math.toRadians(180)))
                                .strafeRight(25)
                                .lineToSplineHeading(new Pose2d(-23.6, -38.6, Math.toRadians(225)))
                                .lineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(180)))
                                .strafeRight(24)
                                .build()
                );

        RoadRunnerBotEntity secondBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -62, Math.toRadians(180)))
                                .strafeRight(25)
                                .lineToSplineHeading(new Pose2d(2, -38.6, Math.toRadians(315)))
                                .lineToSplineHeading(new Pose2d(10, -62, Math.toRadians(0)))
                                .forward(32)
                                .strafeLeft(23)
                                .forward(15)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .addEntity(secondBot)
                .start();
    }
}