package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueInnerCenter {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 11.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 62, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(23, 34, Math.toRadians(-90)),Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(49, 37, Math.toRadians(-180)), Math.toRadians(-90))
//                                .splineToLinearHeading(new Pose2d(49, 35, Math.toRadians(-180)),Math.toRadians(-90))
//                                .splineToLinearHeading(new Pose2d(14, 58, Math.toRadians(180)), Math.toRadians(90))
//                                .splineToLinearHeading(new Pose2d(24, 58, Math.toRadians(180)), Math.toRadians(138))
                                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                                .lineToLinearHeading(new Pose2d(-34, 58, Math.toRadians(-180)))
                                .splineToLinearHeading(new Pose2d(-56, 36, Math.toRadians(-180)), Math.toRadians(-180))
                                .lineToLinearHeading(new Pose2d(-36, 58, Math.toRadians(-180)))
                                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                                .splineToLinearHeading(new Pose2d(50, 30.5, Math.toRadians(-180)), Math.toRadians(-90))
//                       TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(15.81, 65.80, Math.toRadians(-90.00)))
//.splineToLinearHeading(new Pose2d(23.00, 32.24, Math.toRadians(-77.91)), Math.toRadians(-77.91))
//.build();
//drive.setPoseEstimate(untitled0.start());
//        drive.setPoseEstimate(untitled0.start());
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}