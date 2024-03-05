package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(group = "Trial")
@Config
public class ParasCode extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Hanger hanger = null;
    ElapsedTime timer;
    Intake intake = null;
    Outtake outtake = null;
    Drone drone = null;
//    jo paryant lock nhi toh paryant intake bandh naahi
    public static int levelThree = 300, levelTwo = 150, levelOne = 50 ,levelZero = 10, levelFour = 570, intakeLevel = 0;
    public static double kp = 3, ki, kd = 0.4;
//    public static double kp = 3.25, ki, kd = 0.3;
//    public static double kp0 = 2.75, ki0, kd0 = 0.4;
    boolean button_flag_a = false;
    boolean button_flag_y = false;
    int currentstatea = 0;
    int currentstatey = 0;
    int previousstatey = 0;
    int previousstatea = 0;
    //    boolean intakeToggle = false;
    public static double THROTTLE = 1, HEADING = 1, TURN = 1;
    public boolean intakeFLag = true,beamState;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public static double intakeServoPos = 0.68, intakeArmServoPos, hangerServoPos1,hangerServoPos2;
    public static double outtakeWristPos,dronepos, stopperPos , outtakeRightPos, outtakeLeftPos, crabPos;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal, current_errorL, current_errorR;
    private BHI260IMU imu;
    public static boolean outtake_flag, reverse_flag, pixel_drop, slider_flag, close_flag, drone_shoot_flag = false;

    public static boolean start = true;
    public static int count = 0;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    Servo dropLeft, dropRight;

    Slider.SliderState sliderStateValue = Slider.SliderState.LEVEL_ZERO;
    public enum sliderState{
        LEVEL_ZERO,
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        drone = new Drone(hardwareMap, telemetry);
        ElapsedTime timer = new ElapsedTime();

        dropLeft = hardwareMap.get(Servo.class, "dropLeft");
        dropRight = hardwareMap.get(Servo.class, "dropRight");

//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        while (opModeInInit()) {
            Intake.SetIntakePosition(0.2);
            Outtake.setOuttakeArm(0);
            Outtake.outtakeWrist.setPosition(0);
            Outtake.stopper.setPosition(0.5);
            Outtake.crab.setPosition(0.68);
            Drone.initialPos();
            Hanger.setHangerServo(Hanger.down_pos1, Hanger.down_pos2);
            timer.reset();
            dropLeft.setPosition(0.4);
            dropRight.setPosition(1);
            telemetry.addData("crab", Outtake.crab.getPortNumber());
            telemetry.addData("stopper", Outtake.stopper.getPortNumber());
            telemetry.addData("intakeleft", Intake.intakeLeft.getPortNumber());
            telemetry.addData("intakeright", Intake.intakeRight.getPortNumber());
            telemetry.addData("AxonLeft", Outtake.outtakeLeft.getPortNumber());
            telemetry.addData("AxonRIght", Outtake.outtakeRight.getPortNumber());
        }

        TrajectorySequence outtakeInit= drive.trajectorySequenceBuilder(new Pose2d())////
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .addTemporalMarker(()->slider.extendTo(levelZero, output_power))
                .waitSeconds(0.8)//0.3
                .addTemporalMarker(()->Outtake.setOuttakeArm(0))
                .addTemporalMarker(()->Outtake.outtakeWrist.setPosition(0))
                .addTemporalMarker(()->Outtake.stopper.setPosition(0.5))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.5))
                .waitSeconds(0.8)//0.5imp
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .build();

        TrajectorySequence pixelLock= drive.trajectorySequenceBuilder(new Pose2d())////
                .addTemporalMarker(()->Intake.SetIntakePosition(intakeServoPos))
                .addTemporalMarker(()-> Intake.IntakeStart())
                .waitSeconds(0.5)//0.3
                .addTemporalMarker(()->Outtake.crab.setPosition(0.68))
                .addTemporalMarker(()-> Intake.IntakeStop())
                .addTemporalMarker(()->Intake.SetIntakePosition(0.2))
                .build();


        TrajectorySequence outtakePos= drive.trajectorySequenceBuilder(new Pose2d())////
                .addTemporalMarker(()->Intake.IntakeReverse())
                .addTemporalMarker(()->slider.extendTo(levelOne, output_power))
                .addTemporalMarker(()->Intake.SetIntakePosition(0.55))
                .waitSeconds(0.3)
                .addTemporalMarker(()->Outtake.setOuttakeArm(0.99))
                .addTemporalMarker(()-> Outtake.outtakeWrist.setPosition(0.6))
                .addTemporalMarker(()->Outtake.stopper.setPosition(0.5))
                .addTemporalMarker(()->Outtake.crab.setPosition(0.7))
                .waitSeconds(0.5)
                .addTemporalMarker(()->Intake.IntakeStop())
                .build();

        TrajectorySequence IntakeStop = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()-> slider.extendTo(intakeLevel, output_power))
                .addTemporalMarker(()->Intake.IntakeStart())
                .waitSeconds(1)
                .addTemporalMarker(()-> Intake.IntakeStop())
                .addTemporalMarker(() ->Intake.SetIntakePosition(0.2))
                .addTemporalMarker(()-> Outtake.crab.setPosition(0.65))
                        .build();


//        slider.extendTo(levelOne, output_power);
//        Intake.SetIntakePosition(0.55);
//        sleep(300);
//        Outtake.setOuttakeArm(0.95);
//        Outtake.outtakeWrist.setPosition(0.7);
//        Outtake.stopper.setPosition(0.5);
//        Outtake.crab.setPosition(0.7);

        waitForStart();


        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        while (opModeIsActive()) {
            states();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            beamState = beamBreaker.getState();
            if(beamBreaker.getState() == true){
                timer.reset();
            }
//            else{
//                timer.reset();
//            }
            if(timer.milliseconds() == 500){
                beamState = false;
            }
            else{
                beamState = true;
            }
            if(beamState){
                greenLED.setState(false);
                redLED.setState(true);
            }
            if(!beamState){
                greenLED.setState(true);
                redLED.setState(false);
            }

            if(slider_flag){
                THROTTLE = 0.7;
                HEADING = 0.7;
                TURN = 0.7;
            }else{
                THROTTLE = 1;
                HEADING = 1;
                TURN = 1;
            }

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
//                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3));
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
//            );


            drive.update();

            //Slider
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    output_power = lifter_pid(kp, ki, kd, levelTwo);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                    slider.extendTo(levelTwo, output_power);
            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                Hanger.setHangerMotor(Hanger.hangerMotor.getCurrentPosition() + 250);

            }
            //--------------------------------------------------------------------------------------
            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left ){
                count++;
                if(count >= 2){
                    Drone.shootDrone();
                    drone_shoot_flag = true;
                }
            }

            if(currentGamepad1.b && !previousGamepad1.b && beamState && (!slider_flag || start)){
                output_power = lifter_pid(kp, ki, kd, intakeLevel);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(intakeLevel, output_power);
                Intake.SetIntakePosition(intakeServoPos);
                Intake.IntakeStart();
                Outtake.crab.setPosition(0.5);
                start = false;
            }

            else if (currentGamepad1.b && !previousGamepad1.b && !beamState && (!slider_flag || start)) {
//                sleep(500);
                output_power = lifter_pid(kp, ki, kd, intakeLevel);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                drive.followTrajectorySequenceAsync(IntakeStop);
                drive.update();
                timer.reset();
                start = false;
            }
            else if (!currentGamepad1.b && previousGamepad1.b && (!slider_flag || start)) {
                output_power = lifter_pid(kp, ki, kd, intakeLevel);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                drive.followTrajectorySequenceAsync(IntakeStop);
                drive.update();
                start = false;
            }


            if (currentGamepad1.a && !previousGamepad1.a ) {
//                intakeToggle != intakeToggle;
                output_power = lifter_pid(kp, ki, kd, intakeLevel);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(intakeLevel, output_power);
                if(!reverse_flag) {
                    Intake.SetIntakePosition(0.55);
                    Intake.IntakeReverse();
                    reverse_flag = true;
                }
                else{
                    Intake.SetIntakePosition(0.2);
                    Intake.IntakeStop();
                    reverse_flag = false;
                }
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if(!pixel_drop) {
                    Outtake.crab.setPosition(0.65);
                    Outtake.stopper.setPosition(0.25);
                    sleep(500);
                    Outtake.stopper.setPosition(0.5);
                    Outtake.crab.setPosition(0.5);
                    pixel_drop = true;
                }
                else{
                    Outtake.crab.setPosition(0.65);
                    Outtake.stopper.setPosition(0.3);
                    pixel_drop = false;
                }
            }

            if (currentGamepad1.x && !previousGamepad1.x ) {
                output_power = lifter_pid(kp, ki, kd, levelThree);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(levelThree, output_power);
            }
            if (currentGamepad1.y && !previousGamepad1.y ) {
                output_power = lifter_pid(kp, ki, kd, levelFour);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                slider.extendTo(levelFour, output_power);
            }
            if(currentGamepad1.left_trigger >0.3 && previousGamepad1.left_trigger <0.3){
                if(!close_flag) {
//                    Outtake.crab.setPosition(0.5);
                    drive.followTrajectorySequenceAsync(pixelLock);
                    close_flag = true;
                }
                else {
                    Outtake.crab.setPosition(0.5);
                    close_flag = false;
                }
            }
            if(currentGamepad1.right_trigger >0.3 && previousGamepad1.right_trigger <0.3){
                Outtake.crab.setPosition(0.5);
                Outtake.stopper.setPosition(0.3);
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper ) {
                if(!outtake_flag){
                    output_power = lifter_pid(kp, ki, kd, levelOne);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
                    drive.followTrajectorySequenceAsync(outtakePos);
                    drive.update();
                    outtake_flag = true;
                    slider_flag = true;
                }
                else{
                    intakeFLag = true;
                    output_power = lifter_pid(kp, ki, kd, levelZero);
                    if (output_power > 0.9) {
                        output_power = 1;
                    } else if (output_power < 0.2) {
                        output_power = 0;
                    }
//                    Intake.SetIntakePosition(0.55);
//                    slider.extendTo(levelZero, output_power);
//                    sleep(500);
//                    Outtake.setOuttakeArm(0);
//                    Outtake.outtakeWrist.setPosition(0);
//                    Outtake.stopper.setPosition(0.5);
//                    Outtake.crab.setPosition(0.5);
//                    sleep(800);
//                    Intake.SetIntakePosition(0.2);
                    drive.followTrajectorySequenceAsync(outtakeInit);
                    drive.update();
                    outtake_flag = false;
                    slider_flag = false;
                }
            }
            //        Hang Servo
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right && drone_shoot_flag){
//                    schedule(new HangerCommand(hanger, Hanger.HangerState.HANG_SERVO));
                Hanger.setHangerServo(Hanger.hang_pos1,Hanger.hang_pos2);
            }

//        Hang Motor
            if(currentGamepad1.back && !previousGamepad1.back){
                Hanger.setHangerMotor(Hanger.hang_count);
            }
            //Operator commands



//        Hanger Down Motor
            if(currentGamepad2.back && !previousGamepad2.back){
                Hanger.setHangerMotor(Hanger.down_count);
            }

            telemetry.addData("Flag", intakeFLag);
            telemetry.addData("beam Breaker State", beamBreaker.getState());
            telemetry.addData("Slider Count One", Slider.sliderRight.getCurrentPosition());
            telemetry.addData("Slider Count Two", Slider.sliderLeft.getCurrentPosition());
            telemetry.addData("Slider One", Slider.sliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Slider Two", Slider.sliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Intake current", Intake.Intake.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("SLider State", sliderState);
//            telemetry.addData("Drone Servo", Drone.droneServo.getPosition());
            telemetry.update();
        }
    }

    public double lifter_pid(double kp_lifter, double ki_lifter, double kd_lifter, int target)
    {
        lifter_posL = Slider.sliderRight.getCurrentPosition();
        lifter_posR = Slider.sliderLeft.getCurrentPosition();

//        buffer code
        error_lifter = target - lifter_posL;
        error_diff = error_lifter - errorprev ;
        error_int = error_lifter + errorprev;
        current_errorL = (error_int < 5) ? 0 : error_int;
        error_int = current_errorL;
        output_lifter = kp_lifter*error_lifter + kd_lifter*error_diff +ki_lifter*error_int;

        error_lifterR = target - lifter_posR;
        error_diffR = error_lifterR - errorprevR - 5;
        error_intR = error_lifterR + errorprevR;
        current_errorR = (error_intR < 5) ? 0 : error_intR;
        error_intR = current_errorR;
        output_lifterR = kp_lifter*error_lifterR + kd_lifter*error_diffR +ki_lifter*error_intR;

        errorprev = error_lifter;
        errorprevR = error_lifterR;

        return Math.abs(output_lifter);
    }
    public void states() {
        if (gamepad1.left_bumper) {
            currentstatea = 1;
        }
        else {
            currentstatea = 0;
        }
        if (gamepad1.y) {
            currentstatey = 1;
        }
        else {
            currentstatey = 0;
        }
        if (previousstatea != currentstatea && currentstatea == 1) {
            if (button_flag_a)
            {
                button_flag_a = false;
//                telemetry.addData("Check", 10);
//                motor_1.setPower(0.5);
            }
            else {
                button_flag_a = true;
//                telemetry.addData("Check", 20);
//                motor_1.setPower(0);
            }
        }
        previousstatea = currentstatea;
        if (previousstatey != currentstatey && currentstatey == 1) {
            if (button_flag_y)
            {
                button_flag_y = false;
//                telemetry.addData("Check", 10);
//                motor_1.setPower(0.5);
            }
            else {
                button_flag_y = true;
//                telemetry.addData("Check", 20);
//                motor_1.setPower(0);
            }
        }
        previousstatey = currentstatey;

    }
}



