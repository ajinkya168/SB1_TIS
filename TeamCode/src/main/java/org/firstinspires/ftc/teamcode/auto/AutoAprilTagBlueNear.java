package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.google.bcks.ftcrobotcontroller.runtime.AprilTagProcessors;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Globals;
import org.firstinspires.ftc.teamcode.vision.Location;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoAprilTagBlueNear extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camer
    private PropPipeline propPipeline;
    private VisionPortal portal;
    VisionPortal visionPortal;
    public AprilTagDetection desiredTag = null;

    private Location randomization;

    SampleMecanumDrive drive;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;

    Slider slider = null;
    Hanger hanger = null;
    ElapsedTime timer;
    Intake intake = null;
    public static double kp = 3, ki, kd = 0.4;
    Outtake outtake = null;
    Drone drone = null;
    Servo dropLeft, dropRight;

    boolean left_flag, center_flag = false;
    public double DESIRED_DISTANCE = 5;
    public double XCurrentC = 30;
    double YcurrentC = 43.5; //23.5, 43.4
    double  HcurrentC = Math.toRadians(-180);
    public double XCurrentL = 30;
    double YcurrentL = 50; //23.5, 43.4
    double  HcurrentL = Math.toRadians(-180);
    public double XCurrentR = 30;
    double YcurrentR = 28; //23.5, 43.4
    double  HcurrentR = Math.toRadians(-180);
    public double XTemp;
    double YTemp; //23.5, 43.4
    double  HTemp;
    private AprilTagProcessor aprilTag;


    double SPEED_GAIN  = 0.12 ;//0.04; // TODO ==== CHANGE THESE VALUES ====  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN = 0.03;//0.015;   // TODO ==== CHANGE THESE VALUES ====  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   = 0.005 ;//0.03 ;   // TODO ==== CHANGE THESE VALUES ==== Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    double speed = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.3;
    int detectionId = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        dropLeft = hardwareMap.get(Servo.class, "dropLeft");
        dropRight = hardwareMap.get(Servo.class, "dropRight");
        slider = new Slider(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        drone = new Drone(hardwareMap, telemetry);
        Outtake.crab.setPosition(0.68);
        Outtake.stopper.setPosition(0.5);
        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(-90)); // original
        drive.setPoseEstimate(startPose);
        Drone.initialPos();
        Hanger.setHangerServo(Hanger.down_pos1, Hanger.down_pos2);

        dropRight.setPosition(0.9);
        dropLeft.setPosition(0.9);

        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(20.6, 34, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.2)
                .splineToLinearHeading(new Pose2d(23, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    output_power = lifter_pid(kp, ki, kd, 60);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                    slider.extendTo(20, output_power);
                })
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.55))
                .splineToLinearHeading(new Pose2d(49, 37, Math.toRadians(-180)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.6))
                .addTemporalMarker(() -> {
                    output_power = lifter_pid(kp, ki, kd, 0);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                })
                .addTemporalMarker(() -> slider.extendTo(0, output_power))
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(46, 37, Math.toRadians(-180)))
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(46, 12, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-60.5, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.52))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.54))
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.IntakeStop())
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-60.5, 13, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.57))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.61))
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.IntakeStop())
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
//                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
//                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
//                .waitSeconds(0.25)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(28, 52, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(32, 28, Math.toRadians(-180)),Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    output_power = lifter_pid(kp, ki, kd, 60);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                    slider.extendTo(20, output_power);
                })
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.55))
                .lineToLinearHeading(new Pose2d(36,  28, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(52, 45, Math.toRadians(-180)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.6))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(1)
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.6))
                .addTemporalMarker(() -> {
                    output_power = lifter_pid(kp, ki, kd, 0);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                })
                .addTemporalMarker(() -> slider.extendTo(0, output_power))
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(45,41, Math.toRadians(-180)))
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(46, 12, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-60.5, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.52))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.54))
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.IntakeStop())
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(-60.5, 13, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.57))
                .waitSeconds(0.1)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.61))
                .waitSeconds(0.8)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
                .addTemporalMarker(()->Intake.IntakeStop())
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
//                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
//                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
//                .waitSeconds(0.25)
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(20, 32, Math.toRadians(-90)),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(14.5, 36, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(7.5, 28, Math.toRadians(-180.00)))
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(30, 28, Math.toRadians(-180)))
                .build();




//                .addTemporalMarker(()->Intake.SetIntakePosition(0.52))
//                .waitSeconds(0.1)
//                .addTemporalMarker(()->Intake.IntakeStart())
//                .waitSeconds(0.8)
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.54))
//                .waitSeconds(0.8)
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
//                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
//                .addTemporalMarker(()->Intake.IntakeStop())
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
//                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
//                .waitSeconds(0.7)
//                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
//                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
//                .waitSeconds(0.3)
//                .lineToLinearHeading(new Pose2d(-60.5, 13, Math.toRadians(-180)))
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.57))
//                .waitSeconds(0.1)
//                .addTemporalMarker(()->Intake.IntakeStart())
//                .waitSeconds(0.8)
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.61))
//                .waitSeconds(0.8)
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
//                .lineToLinearHeading(new Pose2d(50, 12, Math.toRadians(-180)))
//                .addTemporalMarker(()->Intake.IntakeStop())
//                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
//                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
//                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.7))
//                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
//                .waitSeconds(0.15)
//                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
//                .waitSeconds(0.1)
//                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
//                .waitSeconds(0.25)
//                .build();



        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getLocation());
            if(gamepad1.x ){
                left_flag = true;
                telemetry.addLine("x pressed");
            }
            else if(gamepad1.y ){
                center_flag = true;
                telemetry.addLine("y pressed");
            }
            else if(gamepad1.b){
//                drive.followTrajectorySequence(right);
                telemetry.addLine("b pressed");
            }
            telemetry.update();
        }

        randomization = propPipeline.getLocation();
        portal.close();

        switch (randomization) {
            case LEFT:
//                drive.followTrajectorySequence(left);
                left_flag = true;
                break;
            case RIGHT:
//                drive.followTrajectorySequence(right);
                break;
            case CENTER:
//                drive.followTrajectorySequence(center);
                center_flag = true;
        }

        waitForStart();
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 22);
        telemetry.addData("Position", randomization);
        telemetry.update();

        if(left_flag){
            drive.followTrajectorySequence(left);
        }
        else if(center_flag){
            drive.followTrajectorySequence(center);
        }
        else{

            drive.followTrajectorySequence(right1);
            runAprilTag(3);
            TrajectorySequence right2 = drive.trajectorySequenceBuilder(new Pose2d(XTemp, YTemp, HTemp))
                    .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                        output_power = lifter_pid(kp, ki, kd, 60);
                        if (output_power > 0.9) {
                            output_power = 1;
                        } else if (output_power < 0.2) {
                            output_power = 0;
                        }
                        slider.extendTo(20, output_power);
                    })
                    .addTemporalMarker(() -> Intake.SetIntakePosition(0.45))
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                    .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                    .waitSeconds(0.7)
                    .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> Intake.SetIntakePosition(0.6))
                    .addTemporalMarker(() -> {
                        output_power = lifter_pid(kp, ki, kd, 0);
                        if (output_power > 0.9) {
                            output_power = 1;
                        } else if (output_power < 0.2) {
                            output_power = 0;
                        }
                    })
                    .addTemporalMarker(() -> slider.extendTo(0, output_power))
                    .addTemporalMarker(() -> Outtake.setOuttakeArm(0))
                    .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                    .waitSeconds(0.3)
                    .lineToLinearHeading(new Pose2d(46, 12, Math.toRadians(-180)))
                    .build();

            TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                    // .lineToLinearHeading(new Pose2d(46, 28.5, Math.toRadians(-180)))
                    .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                    .lineToLinearHeading(new Pose2d(-45, 12, Math.toRadians(-180)))
                    .turn(Math.toRadians(-40))
                    .build();

            drive.followTrajectorySequence(right2);
            drive.followTrajectorySequence(right3);
            runAprilTag(10);

        }

        drive.update();
    }

    public double lifter_pid(double kp_lifter, double ki_lifter, double kd_lifter, int target)
    {
        lifter_posL = Slider.sliderRight.getCurrentPosition();
        lifter_posR = Slider.sliderLeft.getCurrentPosition();

        error_lifter = target - lifter_posL;
        error_diff = error_lifter - errorprev;
        error_int = error_lifter + errorprev;
        output_lifter = kp_lifter*error_lifter + kd_lifter*error_diff +ki_lifter*error_int;

        error_lifterR = target - lifter_posR;
        error_diffR = error_lifterR - errorprevR;
        error_intR = error_lifterR + errorprevR;
        output_lifterR = kp_lifter*error_lifterR + kd_lifter*error_diffR +ki_lifter*error_intR;

        errorprev = error_lifter;
        errorprevR = error_lifterR;
        return Math.abs(output_lifter);
    }


    public void runAprilTag(int DESIRED_TAG_ID){

//        initAprilTag();
        TrajectorySequence ResetTrajCenter = drive.trajectorySequenceBuilder(new Pose2d(XCurrentC, YcurrentC, HcurrentC))
                .lineToLinearHeading(new Pose2d(XCurrentC+0.1, YcurrentC+0.1, HcurrentC ))
                .build();
        TrajectorySequence ResetTrajLeft = drive.trajectorySequenceBuilder(new Pose2d(XCurrentL, YcurrentL, HcurrentL))
                .lineToLinearHeading(new Pose2d(XCurrentL+0.1, YcurrentL+0.1, HcurrentL ))
                .build();
        TrajectorySequence ResetTrajRight = drive.trajectorySequenceBuilder(new Pose2d(XCurrentR, YcurrentR, HcurrentR))
                .lineToLinearHeading(new Pose2d(XCurrentR+0.1, YcurrentR+0.1, HcurrentR ))
                .build();



        while (opModeIsActive()) {

            //// TODO TRYING TO DETECT THE APRIL TAG
            drive.update();
            desiredTag = null;

            try {
                // Step through the list of detected tags and look for a matching tag

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();


                for (AprilTagDetection detection : currentDetections) {
                    telemetry.addLine("Inside for block");
                    telemetry.update();
                    detectionId = detection.id;
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        telemetry.addLine("Inside if block");
                        telemetry.update();
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || detection.id == DESIRED_TAG_ID) {
                            desiredTag = detection;
                            break;  //
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        telemetry.update();
                        if(DESIRED_TAG_ID == 3 ){
                            drive.followTrajectorySequence(ResetTrajRight);
                            drive.update();

                        }
                        else  if(DESIRED_TAG_ID == 2){
                            drive.followTrajectorySequence(ResetTrajCenter);
                            drive.update();

                        }
                        else if(DESIRED_TAG_ID == 1){
                            drive.followTrajectorySequence(ResetTrajLeft);
                            drive.update();
                        }
                    }
                }



                //// TODO GETTING THE ERRORS FROM APRIL TAGS

                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                speed = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


                moveRobot(speed, strafe, turn);

                sleep(10);
                if (rangeError < 0.5) {
                    moveRobot(0, 0, 0);
                    break;
                }

            }
            catch (Exception e)
            {
                //  TODO THESE TRAJECTORIES WILL EXECUTE IF THE CAMERA DIDN'T WORKED
                telemetry.addLine("In Catch Block");
                telemetry.update();
                if(DESIRED_TAG_ID == 3 ){
                    drive.followTrajectorySequence(ResetTrajRight);
                    drive.update();

                }
                else if(DESIRED_TAG_ID == 2 ){
                    drive.followTrajectorySequence(ResetTrajCenter);
                    drive.update();

                }
                else if(DESIRED_TAG_ID == 1){
                    drive.followTrajectorySequence(ResetTrajLeft);
                    drive.update();


                }

            }
        }
        drive.update();
        XTemp = drive.getPoseEstimate().getX();
        YTemp = drive.getPoseEstimate().getY();
        HTemp = drive.getPoseEstimate().getHeading();
        drive.update();
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y +yaw;
        double rightFrontPower   =  x +y -yaw;
        double leftBackPower     =  x +y +yaw;
        double rightBackPower    =  x -y -yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        drive.leftFront.setPower(-leftFrontPower);
        drive.rightFront.setPower(-rightFrontPower);
        drive.leftRear.setPower(-leftBackPower);
        drive.rightRear.setPower(-rightBackPower);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

}
