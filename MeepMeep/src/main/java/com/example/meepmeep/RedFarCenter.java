package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFarCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 11.75)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-24-14, -62, Math.toRadians(90)))
                                        .splineToLinearHeading(new Pose2d(-24-12, -33, Math.toRadians(90)),Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(-24-14, -45, Math.toRadians(90)))
                                        .splineToLinearHeading(new Pose2d(-56, -10, Math.toRadians(-180)), Math.toRadians(90))
                                        .lineToLinearHeading(new Pose2d(54, -12, Math.toRadians(-180)))
                                        .lineToLinearHeading(new Pose2d(56, -32, Math.toRadians(-180)))
                                        .lineToLinearHeading(new Pose2d(56, -43.2, Math.toRadians(-180)))

                                        .lineToLinearHeading(new Pose2d(54, -12, Math.toRadians(-180)))
                                        .lineToLinearHeading(new Pose2d(-56, -10, Math.toRadians(-180)))
////                                .lineToLinearHeading(new Pose2d(23, 40, Math.toRadians(-180.00)))
//                                        .lineToLinearHeading(new Pose2d(23, -42, Math.toRadians(90)))
//                                        .splineToLinearHeading(new Pose2d(48, -32, Math.toRadians(-180)),Math.toRadians(90))
//

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 11.75)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(14, -62, Math.toRadians(90)))
//                                .splineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)),Math.toRadians(90))
////                                .lineToLinearHeading(new Pose2d(23, 40, Math.toRadians(-180.00)))
//                                .back(10)
//                                .splineToLinearHeading(new Pose2d(48, -34, Math.toRadians(180)),Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}