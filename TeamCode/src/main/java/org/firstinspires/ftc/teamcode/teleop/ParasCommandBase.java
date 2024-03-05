package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.DroneCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.HangerCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.IntakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.SliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.Back2Pos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.BothPixelDrop;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.DropPos;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.IntakeStart;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.IntakeStop;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.LowerPixelDrop;
import org.firstinspires.ftc.teamcode.commandbase.teleopCommands.UpperPixelDrop;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Trial")
@Config
public class ParasCommandBase extends CommandOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Hanger hanger = null;
    ElapsedTime timer;
    Intake intake = null;
    Outtake outtake = null;
    Drone drone = null;
    int levelTwo =180,levelOne = 130,levelZero = 0;
    public static double kp = 3, ki, kd = 0.4;
    public static double kp0 = 2, ki0, kd0 = 0;
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
            lifter_posL = 0, lifter_posR = 0, error_lifter, error_diff, error_int, error_lifterR, error_diffR, error_intR, errorprev, errorprevR, output_lifter, output_lifterR, output_power, target, dropVal;
    private BHI260IMU imu;
    public static boolean outtake_flag, reverse_flag, pixel_drop, dpadl, dpadr = false;

    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    DigitalChannel beamBreaker;

    @Override
    public void initialize() {
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        slider = new Slider(hardwareMap, telemetry);
//        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        drone = new Drone(hardwareMap, telemetry);
        timer = new ElapsedTime();

        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");
        beamBreaker.setMode(DigitalChannel.Mode.INPUT);

//        Intake.SetIntakePosition(0.2);
        schedule(new IntakeWristCommand(intake, Intake.IntakeWristState.INIT));
//        Outtake.setOuttakeArm(0);
        schedule(new OuttakeArmCommand(outtake, Outtake.OuttakeArmState.INIT));
//        Outtake.outtakeWrist.setPosition(0);
        schedule(new OuttakeWristCommand(outtake, Outtake.OuttakeWristState.INIT));
//        Outtake.stopper.setPosition(0.5);
        schedule(new StopperCommand(outtake, Outtake.StopperState.CLOSE));
//        Outtake.crab.setPosition(0.5);
        schedule(new CrabCommand(outtake, Outtake.CrabState.NEUTRAL));
//        Drone.initialPos();
        schedule(new DroneCommand(drone, Drone.DroneState.INIT));
        timer.reset();
    }

    @Override
    public void run() {
        super.run();
//        Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
//                Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3));
//
//        drive.setWeightedDrivePower(
//                new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING)
//        );
//        drive.update();

        if(gamepad1.a){
            telemetry.addLine("pressed");
        }

//        Slider up level one
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            output_power = lifter_pid(kp, ki, kd, levelOne);
            if (output_power > 0.9) {
                output_power = 1;
            } else if (output_power < 0.2) {
                output_power = 0;
            }
//            slider.extendTo(levelOne, output_power);
            schedule(new SliderCommand(slider, Slider.SliderState.LEVEL_ONE));
        }

//        Slider up level two
        if (currentGamepad1.y && !previousGamepad1.y ) {
            output_power = lifter_pid(kp, ki, kd, levelTwo);
            if (output_power > 0.9) {
                output_power = 1;
            } else if (output_power < 0.2) {
                output_power = 0;
            }
//            slider.extendTo(levelTwo, output_power);
            schedule(new SliderCommand(slider, Slider.SliderState.LEVEL_TWO));
        }

//        toggle for Outtake position
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper ) {
            if(!outtake_flag){
                schedule(new DropPos(intake, outtake));
                outtake_flag = true;
            }
            else{
                intakeFLag = true;
                output_power = lifter_pid(kp, ki, kd, levelZero);
                if (output_power > 0.9) {
                    output_power = 1;
                } else if (output_power < 0.2) {
                    output_power = 0;
                }
                schedule(new Back2Pos(intake, outtake, slider));
                outtake_flag = false;
            }
        }

//      Pixel Grip
        if(currentGamepad1.left_trigger >0.3 && previousGamepad1.left_trigger <0.3){
//            Outtake.crab.setPosition(0.65);
            schedule(new CrabCommand(outtake, Outtake.CrabState.GRAB));
        }

//      Pixel drop toggle
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            if(!pixel_drop) {
                schedule(new LowerPixelDrop(intake, outtake));
                pixel_drop = true;
            }
            else{
                schedule(new UpperPixelDrop(intake, outtake));
                pixel_drop = false;
            }
        }

//        Both Pixel drop
        if(currentGamepad1.right_trigger >0.3 && previousGamepad1.right_trigger <0.3){
//            Outtake.crab.setPosition(0.65);
            schedule(new BothPixelDrop(intake, outtake));
        }

//        Drone Shoot
        if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
            schedule(new DroneCommand(drone, Drone.DroneState.SHOOT));
        }

//        Intake ClockWise
        if(currentGamepad1.b && !previousGamepad1.b && beamState){
            schedule(new IntakeStart(intake, outtake));
            Intake.IntakeStart();
        }
        else if (currentGamepad1.b && !previousGamepad1.b && !beamState) {
            schedule(new IntakeStop(intake, outtake));
            Intake.IntakeStop();
            timer.reset();
        }
        else if (!currentGamepad1.b && previousGamepad1.b) {
            schedule(new IntakeStop(intake, outtake));
            Intake.IntakeStop();
        }

//        Intake AntiClockwise
        if (currentGamepad1.a) {
            if(!reverse_flag) {
                Intake.IntakeReverse();
                schedule(new WaitCommand(300));
                reverse_flag = true;
            }
            else{
                Intake.IntakeStop();
                reverse_flag = false;
            }
        }

//        Hang Servo
        if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            schedule(new HangerCommand(hanger, Hanger.HangerState.HANG_SERVO));
        }

//        Hang Motor
        if(currentGamepad1.start && !previousGamepad1.start){
            schedule(new HangerCommand(hanger, Hanger.HangerState.HANG_MOTOR));
        }

//        Hanger Down Servo
        if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
            schedule(new HangerCommand(hanger, Hanger.HangerState.DOWN_SERVO));
        }

//        Hanger Down Motor
        if(currentGamepad2.start && !previousGamepad2.start){
            schedule(new HangerCommand(hanger, Hanger.HangerState.DOWN_MOTOR));
        }

        telemetry.addData("Flag", intakeFLag);
        telemetry.addData("beam Breaker State", beamBreaker.getState());

        telemetry.addData("Slider Count One", Slider.sliderRight.getCurrentPosition());
        telemetry.addData("Slider Count Two", Slider.sliderLeft.getCurrentPosition());
        telemetry.addData("Slider One", Slider.sliderRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Slider Two", Slider.sliderLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake current", Intake.Intake.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Drone Servo", Drone.droneServo.getPosition());
        telemetry.update();
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



