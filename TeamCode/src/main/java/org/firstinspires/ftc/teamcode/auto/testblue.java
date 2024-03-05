//TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
//        .splineToLinearHeading(new Pose2d(-24-12, -33, Math.toRadians(90)),Math.toRadians(90))
////                .waitSeconds(5)
//        .addTemporalMarker(()->{
//        dropLeft.setPosition(0.6);
//        })
//        .waitSeconds(0.4)
//        .lineToLinearHeading(new Pose2d(-24-14, -45, Math.toRadians(90)))
//        .splineToLinearHeading(new Pose2d(-56, -10, Math.toRadians(-180)), Math.toRadians(90))
//        //stack pick
//        .addTemporalMarker(()->Intake.SetIntakePosition(0.47))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Intake.IntakeStart())
//        .waitSeconds(1)
//        .lineToLinearHeading(new Pose2d(50, -12, Math.toRadians(-180)))
//        .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
//        Outtake.crab.setPosition(0.68);
//        output_power = lifter_pid(kp, ki, kd, 35);
//        if (output_power > 0.9) {
//        output_power = 1;
//        } else if (output_power < 0.2) {
//        output_power = 0;
//        }
//        slider.extendTo(35, output_power);
//        })
//        .addTemporalMarker(()->Intake.IntakeStop())
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Intake.SetIntakePosition(0.6))
//        .addTemporalMarker(()->Intake.IntakeReverse())
//        .lineToLinearHeading(new Pose2d(56, -34, Math.toRadians(-180)))
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0.6))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0.7))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.outtakeWrist.setPosition(0.7))
//        .waitSeconds(0.5)
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0.85))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0.9))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0.98))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
//        .waitSeconds(0.1)
//        .addTemporalMarker(()->Outtake.stopper.setPosition(0.25))
//        .waitSeconds(1)
//        .addTemporalMarker(()->{
//        output_power = lifter_pid(kp, ki, kd, 150);
//        if (output_power > 0.9) {
//        output_power = 1;
//        } else if (output_power < 0.2) {
//        output_power = 0;
//        }
//        slider.extendTo(150, output_power);
//        })
//        .waitSeconds(0.5)
//        .addTemporalMarker(()->Intake.IntakeStop())
//        .lineToLinearHeading(new Pose2d(56, -27, Math.toRadians(-180)))
//        .addTemporalMarker(()->Outtake.outtakeWrist.setPosition(0.7))
//        .waitSeconds(0.5)
//        .addTemporalMarker(()->Outtake.crab.setPosition(0.5))
//        .waitSeconds(0.2)
//        .addTemporalMarker(()->Outtake.stopper.setPosition(0.25))
//        .waitSeconds(0.7)
//        .lineToLinearHeading(new Pose2d(50, -27, Math.toRadians(-180)))
//        .addTemporalMarker(()->{ output_power = lifter_pid(kp, ki, kd, 0);
//        if (output_power > 0.9) {
//        output_power = 1;
//        } else if (output_power < 0.2) {
//        output_power = 0;
//        }
//        })
//        .addTemporalMarker(()->slider.extendTo(0, output_power))
//        .addTemporalMarker(()->Outtake.setOuttakeArm(0))
//        .addTemporalMarker(()->Outtake.outtakeWrist.setPosition(0))
//        .waitSeconds(0.3)
//        .lineToLinearHeading(new Pose2d(55, -9, Math.toRadians(-180)))
//        .lineToLinearHeading(new Pose2d(60, -9, Math.toRadians(-180)))
//        .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
//        .build();