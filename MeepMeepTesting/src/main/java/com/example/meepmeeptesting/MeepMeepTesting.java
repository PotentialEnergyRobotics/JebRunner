package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -66, Math.toRadians(135)))
                                .lineTo(new Vector2d(-36,-36))
                                .lineTo(new Vector2d(-24,-36))
                                .lineTo(new Vector2d(-36,-36))
                                .lineTo(new Vector2d(-36,-23))
                                .splineToLinearHeading(new Pose2d(-54, -12, Math.toRadians(225)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-24,-12, Math.toRadians(135)))
                                .lineToLinearHeading(new Pose2d(-54,-12, Math.toRadians(225)))
                                .lineTo(new Vector2d(-48,-12))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-36,-24), Math.toRadians(-90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}