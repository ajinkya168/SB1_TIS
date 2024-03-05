package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(group = "Trial")
@Disabled
public class autoBlue extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    List<LynxModule> allHubs = null;

    private VisionPortal visionPortal;

    double x;
    double y;
    String propPosition = " ";

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

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        dropLeft = hardwareMap.get(Servo.class, "dropLeft");
        slider = new Slider(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        drone = new Drone(hardwareMap, telemetry);
        allHubs = hardwareMap.getAll(LynxModule.class);
        Outtake.crab.setPosition(0.68);
        Outtake.stopper.setPosition(0.5);
        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(-90)); // original
        drive.setPoseEstimate(startPose);
        Drone.initialPos();
        Hanger.setHangerServo(Hanger.down_pos1, Hanger.down_pos2);

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(20, 34, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.1)
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
                .splineToLinearHeading(new Pose2d(50, 37, Math.toRadians(-180)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.6))
                .waitSeconds(0.2)
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
                .lineToLinearHeading(new Pose2d(52, 60, Math.toRadians(-180)))
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(20, 32, Math.toRadians(-90)),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(12, 34, Math.toRadians(0)), Math.toRadians(-90))
//                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(10, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(10, 40, Math.toRadians(45)))
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
                .splineToLinearHeading(new Pose2d(50, 41, Math.toRadians(-180)), Math.toRadians(-180))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.6))
                .waitSeconds(0.2)
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
                .lineToLinearHeading(new Pose2d(52, 60, Math.toRadians(-180)))
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(20, 32, Math.toRadians(-90)),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(14, 36, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(7, 28, Math.toRadians(-180.00)))
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(20, 36, Math.toRadians(-180)))
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
                .splineToLinearHeading(new Pose2d(51, 28.5, Math.toRadians(-180)), Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.6))
                .waitSeconds(0.2)
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
                .lineToLinearHeading(new Pose2d(52, 60, Math.toRadians(-180)))
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .build();

        dropLeft.setPosition(0.9);
        initTfod();
        visionPortal.setProcessorEnabled(tfod, true);

        while (isStopRequested() == false && isStarted() == false )   //&& ParkingZone == "None"
        {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
            if (currentRecognitions.size() != 0) {
                boolean objectFound = false;
                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;

                    objectFound = true;
                    telemetry.addLine("beacon");
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    telemetry.update();
                    break;
                }

                if(objectFound){
//                    Adjust values according to your bot and camera position
                    if( x>250 && x<=530){
                        propPosition  = "left";
                    }
                    else if(x>=550 && x<=700){
                        propPosition = "center";
                    }
                    else if(x>=800) {
                        propPosition = "right";
                    }
                }
                else{
                    telemetry.addLine("Don't see the beacon :(");
                }

            }
            telemetry.update();

        }

        waitForStart();
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.close();

        //trajectory follow
//        drive.followTrajectorySequence(right);
//        drive.update();
        telemetry.addData("position",propPosition);
        if (propPosition.equals("left")) {
            drive.followTrajectorySequence(left);
        } else if (propPosition.equals("right")) {
            drive.followTrajectorySequence(right);
        } else {
            drive.followTrajectorySequence(center);
        }
        drive.update();

//        Then follow your cycle trajectories

        telemetry.addData("x", startPose.getX());
        telemetry.addData("y", startPose.getY());
        telemetry.addData("heading", startPose.getHeading());
        telemetry.update();
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
//                .setNumDetectorThreads()

                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));
//        builder.setCamera()

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

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
}
