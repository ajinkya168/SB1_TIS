package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(group = "Cycle")
public class autoBlueNearCycle extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    List<LynxModule> allHubs = null;
    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;

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

    boolean left_flag, center_flag = false;

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
        allHubs = hardwareMap.getAll(LynxModule.class);
        Outtake.crab.setPosition(0.68);
        Outtake.stopper.setPosition(0.5);
        Pose2d startPose = new Pose2d(14, 62, Math.toRadians(-90)); // original
        drive.setPoseEstimate(startPose);
        Drone.initialPos();
        Hanger.setHangerServo(Hanger.down_pos1, Hanger.down_pos2);

        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        dropRight.setPosition(0.9);
        dropLeft.setPosition(0.9);

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
                .waitSeconds(0.1)
                .splineToLinearHeading(new Pose2d(23, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .addTemporalMarker( () -> {
                    output_power = lifter_pid(kp, ki, kd, 60);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                    slider.extendTo(20, output_power);
                })
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.55))
                .splineToLinearHeading(new Pose2d(49.5, 37, Math.toRadians(-180)), Math.toRadians(-90))
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.2)
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
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0))
                .waitSeconds(0.3)
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-34, 58, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(-61, 32.7, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(() -> {
                    output_power = lifter_pid(kp, ki, kd, 30);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                })
                .addTemporalMarker(() -> slider.extendTo(30, output_power))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.47))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(1)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.53))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-36, 58, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(50, 30.5, Math.toRadians(-180)), Math.toRadians(-90))
                .addTemporalMarker(()->Intake.IntakeStop())
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(32, 25, Math.toRadians(-180)),Math.toRadians(-90))
                .addTemporalMarker(() -> Outtake.setOuttakeArm(0.98))
                .addTemporalMarker(() -> Outtake.outtakeWrist.setPosition(0.7))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> Outtake.crab.setPosition(0.5))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Outtake.stopper.setPosition(0.3))
                .waitSeconds(1)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(20, 32, Math.toRadians(-90)),Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(32, 28, Math.toRadians(-180)),Math.toRadians(-90))
//                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(28, 52, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(32, 28, Math.toRadians(-180)),Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.5)
                .addTemporalMarker( () -> {
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
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-34, 58, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(-61, 32.7, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.47))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(2)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.5))
                .lineToLinearHeading(new Pose2d(-34, 55, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(20, 32, Math.toRadians(-90)),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(14.5, 36, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToLinearHeading(new Pose2d(7.5, 28, Math.toRadians(-180.00)))
                .addTemporalMarker(() -> {
                    dropLeft.setPosition(0.6);
                })
                .waitSeconds(0.5)
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
                .splineToLinearHeading(new Pose2d(50.5, 30.5, Math.toRadians(-180)), Math.toRadians(-90))
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
                .addTemporalMarker(() -> Intake.SetIntakePosition(0.2))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-34, 58, Math.toRadians(-180)))
                .splineToLinearHeading(new Pose2d(-61, 32.7, Math.toRadians(-180)), Math.toRadians(-180))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.47))
                .waitSeconds(0.2)
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(2)
                .addTemporalMarker(()->Intake.SetIntakePosition(0.5))
                .lineToLinearHeading(new Pose2d(-34, 58, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(34, 58, Math.toRadians(-180)))
                .build();

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

        telemetry.addData("Position", randomization);
        telemetry.update();

       if(left_flag){
           drive.followTrajectorySequence(left);
       }
       else if(center_flag){
           drive.followTrajectorySequence(center);
       }
       else{
           drive.followTrajectorySequence(right);
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
}
