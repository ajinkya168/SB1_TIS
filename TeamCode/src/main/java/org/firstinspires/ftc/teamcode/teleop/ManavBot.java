package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Trial")
@Config
public class ManavBot extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Hanger hanger = null;
    ElapsedTime timer;
    Intake intake = null;
    Outtake outtake = null;
    Drone drone = null;
    ElapsedTime inputTimer, outputTimer,dropTwoTimer;
    int level =0,levelOne = 400,levelZero = 0;
    public static double kp = 3, ki, kd = 0.4;
    public static double kp0 = 2, ki0, kd0 = 0;
    public static double sliderCounter;
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
    IntakeState inputState = IntakeState.INTAKE_START;
    OuttakeState outputState = OuttakeState.OUTTAKE_START;
    DropOneState dropOneState = DropOneState.DROP_ONE_START;

    DropTwoState dropTwoState = DropTwoState.DROP_TWO_START;
public static int intakeCounter=0;
    public static double intakeServoPos = 0.68, intakeArmServoPos, hangerServoPos1,hangerServoPos2;
    public static double outtakeWristPos,dronepos, stopperPos , outtakeRightPos, outtakeLeftPos, crabPos;
    public static double
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;
    private BHI260IMU imu;
    public enum IntakeState {
        INTAKE_START,
        INTAKE_INTAKE,
        INTAKE_CRAB,
        INTAKE_FINAL
    };
    public enum OuttakeState{
        OUTTAKE_START,
        OUTTAKE_OUTTAKE,
        OUTTAKE_SLIDER,
        OUTTAKE_FINAL
    };
    public enum DropOneState{
        DROP_ONE_START,
        DROP_ONE,
        DROP_ONE_FINAL
    };
    public enum DropTwoState{
        DROP_TWO_START,
        DROP_TWO,
        DROP_TWO_INIT,
        DROP_TWO_SLIDER,
        DROP_TWO_FINAL
    };
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        inputTimer = new ElapsedTime();
        outputTimer = new ElapsedTime();
        dropTwoTimer = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
//        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        drone = new Drone(hardwareMap, telemetry);
        ElapsedTime timer = new ElapsedTime();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
//        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // TODO: reverse any motors using DcMotor.setDirection()
//        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
//        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
//
//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        imu.initialize(parameters);

//        AnalogInput armOneAnalogInput = hardwareMap.get(AnalogInput.class, "armOneAnalogInput");
//        AnalogInput armTwoAnalogInput = hardwareMap.get(AnalogInput.class, "armTwoAnalogInput");
//        AnalogInput intakeArmAnalogInput = hardwareMap.get(AnalogInput.class, "intakeArmAnalogInput");
//        AnalogInput intakeWristAnalogInput = hardwareMap.get(AnalogInput.class, "intakeWristAnalogInput");
//        AnalogInput crankAnalogInput = hardwareMap.get(AnalogInput.class, "crankAnalogInput");
//        AnalogInput wristAnalogInput = hardwareMap.get(AnalogInput.class, "wristAnalogInput");

        DigitalChannel beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

        while (opModeInInit()) {
            Intake.SetIntakePosition(0.2);
            Outtake.setOuttakeArm(0);
            Outtake.outtakeWrist.setPosition(0);
            Outtake.stopper.setPosition(0.5);
            Outtake.crab.setPosition(0.5);
            timer.reset();
        }

        waitForStart();

        while (opModeIsActive()) {
            states();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            beamState = beamBreaker.getState();
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//            if (currentGamepad1.start && !previousGamepad1.start) {
//                imu.resetYaw();
//            }
//
//            // Main teleop loop goes here
//
//            //drivetrain ---------------------------------------------------------------------------
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            leftFront.setPower(frontLeftPower);
//            leftRear.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightRear.setPower(backRightPower);

//            double intakeArmPosition = intakeArmAnalogInput.getVoltage() / 3.3 * 360;
//            double intakeWristPosition = intakeWristAnalogInput.getVoltage() / 3.3 * 360;
//            double crankPosition = crankAnalogInput.getVoltage() / 3.3 * 360;
//            double wristPosition = wristAnalogInput.getVoltage() / 3.3 * 360;
//            double armOnePosition = armOneAnalogInput.getVoltage() / 3.3 * 360;
//            double armTwoPosition = armTwoAnalogInput.getVoltage() / 3.3 * 360;

            // Main teleop loop goes here

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
            );
            drive.update();


            switch (inputState){
                case INTAKE_START:
                    //waiting for input
                    if(currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                        inputTimer.reset();
                        inputState = IntakeState.INTAKE_INTAKE;
                    }
                    break;
                case INTAKE_INTAKE:
                    Intake.SetIntakePosition(intakeServoPos);
                    if (beamBreaker.getState()){
                        Intake.IntakeStart();
                        Outtake.crab.setPosition(0.5);
                            inputState = IntakeState.INTAKE_CRAB;

                    }
                    if (!beamBreaker.getState()){
                        if (inputTimer.milliseconds() >= 2000){ // 500 //800
                            Outtake.crab.setPosition(0.65);
                            Intake.IntakeStop();
                            inputTimer.reset();
                            inputState = IntakeState.INTAKE_CRAB;
                        }
                    }
                    break;

                case INTAKE_CRAB:
                    Outtake.crab.setPosition(0.65);
                    inputTimer.reset();
                    inputState = IntakeState.INTAKE_FINAL;

                    break;
                case INTAKE_FINAL:
                    Intake.SetIntakePosition(0.3);
                    inputState = IntakeState.INTAKE_START;
                    break;
                default:
                    inputState = IntakeState.INTAKE_START;
            }

            switch(outputState)
            {
                case OUTTAKE_START:
                    if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                    outputState = OuttakeState.OUTTAKE_OUTTAKE;}
                    break;
                case OUTTAKE_OUTTAKE:
                    Outtake.setOuttakeArm(0.95);
                    Outtake.outtakeWrist.setPosition(0.7);
                    Outtake.stopper.setPosition(0.5);
                    Outtake.crab.setPosition(0.7);
                    Intake.SetIntakePosition(0.55);
                    if(sliderCounter != 0){
                        outputState = OuttakeState.OUTTAKE_SLIDER;}
                    else
                    {
                        outputState = OuttakeState.OUTTAKE_FINAL;
                    }
                    break;
                case OUTTAKE_SLIDER:
                    if(outputTimer.milliseconds()>=200) {
                        if (sliderCounter == 1) {
                            output_power = lifter_pid(kp, ki, kd, levelOne);
                            if (output_power > 0.9) {
                                output_power = 1;
                            } else if (output_power < 0.2) {
                                output_power = 0;
                            }
                            slider.extendTo(levelOne, output_power);
                        }
                        outputTimer.reset();
                        outputState = OuttakeState.OUTTAKE_FINAL;
                    }
                    break;
                case OUTTAKE_FINAL:
                    outputState = OuttakeState.OUTTAKE_START;
                    break;
                default:
                    outputState = OuttakeState.OUTTAKE_START;
            }
            switch(dropOneState)
            {
                case DROP_ONE_START:
                    if(currentGamepad1.b && !previousGamepad1.b)
                    {
                    dropOneState = DropOneState.DROP_ONE;}
                    break;
                case DROP_ONE:
                    Outtake.stopper.setPosition(0.3);
                    Intake.SetIntakePosition(0.55);
                    dropOneState = DropOneState.DROP_ONE_FINAL;
                    break;
                case DROP_ONE_FINAL:
                    dropOneState = DropOneState.DROP_ONE_START;
                    break;
                default:
                    dropOneState = DropOneState.DROP_ONE_START;
            }
            switch(dropTwoState)
            {
                case DROP_TWO_START:
                    if(currentGamepad1.a && !previousGamepad1.a){
                    dropTwoState = DropTwoState.DROP_TWO;}
                    break;
                case DROP_TWO:
                    Outtake.stopper.setPosition(0.3);
                    Intake.SetIntakePosition(0.55);
                    dropTwoState = DropTwoState.DROP_TWO_INIT;
                    break;
                case DROP_TWO_INIT:
                    Outtake.setOuttakeArm(0);
                    Outtake.outtakeWrist.setPosition(0);
                    Outtake.stopper.setPosition(0.5);
                    Outtake.crab.setPosition(0.5);
                    Intake.SetIntakePosition(0.68);
                    dropTwoState = DropTwoState.DROP_TWO_SLIDER;
                    break;
                case DROP_TWO_SLIDER:
                    if (dropTwoTimer.milliseconds() >= 300){//400
                        output_power = lifter_pid(kp, ki, kd, 0);
                        if (output_power > 0.9) {
                            output_power = 1;
                        } else if (output_power < 0.2) {
                            output_power = 0;
                        }
                        slider.extendTo(0, output_power);
                        dropTwoTimer.reset();
                        dropTwoState = DropTwoState.DROP_TWO_FINAL;

                    }
                    sliderCounter =0;
                    break;
                case DROP_TWO_FINAL:
                    dropTwoState = DropTwoState.DROP_TWO_START;
                    break;
                default:
                    dropTwoState = DropTwoState.DROP_TWO_START;
            }

            //Slider
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && (intakeCounter == 1) ){
                intakeCounter = 0;
            }

            if(gamepad1.left_trigger>=0)
            {
                Intake.IntakeStart();
            }

            if (gamepad1.right_trigger>=0) {
//                intakeToggle != intakeToggle;
                Intake.IntakeReverse();
            }
            else {
                Intake.IntakeStop();
            }


            telemetry.addData("beam Breaker State", beamBreaker.getState());

            telemetry.addData("Slider Count One", Slider.sliderRight.getCurrentPosition());
            telemetry.addData("Slider Count Two", Slider.sliderLeft.getCurrentPosition());
            telemetry.addData("Slider One", Slider.sliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Slider Two", Slider.sliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Intake current", Intake.Intake.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Drone Servo", Drone.droneServo.getPosition());
            telemetry.update();
        }
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
    public void states() {
        if (gamepad1.left_trigger>=0) {
            currentstatea = 1;
        }
        else {
            currentstatea = 0;
        }
        if (gamepad1.right_trigger>=0) {
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



