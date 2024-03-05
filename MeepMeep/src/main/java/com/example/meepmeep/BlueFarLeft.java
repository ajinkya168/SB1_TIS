package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFarLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 11.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-24-14, 62, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(-24-8, 38, Math.toRadians(0)),Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(-24-14, 45, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(-54, 10, Math.toRadians(-180)), Math.toRadians(-90))
                                //stack pick


//                                .lineToLinearHeading(new Pose2d(-24-20, 40, Math.toRadians(-45)))
//                                .lineToLinearHeading(new Pose2d(23, 40, Math.toRadians(-180.00)))

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