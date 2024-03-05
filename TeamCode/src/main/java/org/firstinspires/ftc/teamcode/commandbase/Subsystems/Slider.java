package org.firstinspires.ftc.teamcode.commandbase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slider {
    public static DcMotorEx
            sliderRight = null, sliderLeft = null;
    public static double motorPowerUP = 1, motorPowerDOWN = 1;

    public static int levelZero = 0;
    public static int levelOne = 130;
    public static int levelTwo = 210;
    MotionProfile motionProfilex;
    ElapsedTime timer;
    public double targetX;
    public static double POWER = -1;
    public static boolean powerflag = false;
    public static double maxVel = 500;
    public static double maxAccel = 1000;

    public static double Kp_slider = 0.01;
    public static double Ki_slider = 0;
    public static double Kd_slider = 0;
    public static double Kf_slider = 0;

    public static int levelThree = 290;
    public Slider(HardwareMap hardwareMap, Telemetry telemetry) {
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderMotorOne");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderMotorTwo");

        sliderRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //motor directions
        sliderRight.setDirection(DcMotorEx.Direction.REVERSE);
        sliderLeft.setDirection(DcMotorEx.Direction.REVERSE);

        //reset motor encoders
        sliderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        timer= new ElapsedTime();

//        sliderLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(40.0,0.0,10,0.0);
////        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(3.0, 0.0, 0.4, 0.0);
//        sliderLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        sliderRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//
//        sliderLeft.setTargetPositionTolerance(5);
//        sliderRight.setTargetPositionTolerance(5);
    }

    public enum SliderState{
        LEVEL_ZERO,
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        LEVEL_FOUR

    }
    SliderState sliderState = SliderState.LEVEL_ZERO;

    public void update(SliderState state){
        sliderState = state;
        switch (state){
            case LEVEL_ZERO:
                extendTo(levelZero, motorPowerUP);
                break;
            case LEVEL_ONE:
                extendTo(levelOne, motorPowerUP);
                break;
            case LEVEL_TWO:
                extendTo(levelTwo, motorPowerUP);
                break;
            case LEVEL_THREE:
                extendTo(levelThree, motorPowerUP);
                break;
        }
    }

    public static void IncreaseExtension(double level){
        sliderRight.setTargetPosition((int) level);
        sliderLeft.setTargetPosition((int) level);

        sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        sliderRight.setPower(motorPowerUP);
        sliderLeft.setPower(motorPowerUP);
    }
    public static void DecreaseExtension(double level){
        sliderRight.setTargetPosition((int) level);
        sliderLeft.setTargetPosition((int) level);

        sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        sliderRight.setPower(motorPowerDOWN);
        sliderLeft.setPower(motorPowerDOWN);
    }

    public double getPosition() {
        return sliderLeft.getCurrentPosition();
    }

    public void set(double power) {

        power = Range.clip(power, -1, 1);

        sliderLeft.setPower(power);
        sliderRight.setPower(power);

//        this.telemetry.addData("slider Pow",leftSlider.getPower());
    }


    public static void extendTo(int targetPos,double pow){
        sliderRight.setTargetPosition(targetPos);
        sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderRight.setPower(pow);

        sliderLeft.setTargetPosition(targetPos);
        sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sliderLeft.setPower(pow);
    }

    public void extendToHome(){
        extendTo(0,0.8);
    }

    //Motion Profiling

    public void goTo(double x) {
//        if(x > 0){
//            sliderState = "OPEN";
//        }
//        else if (x == 0){
//            sliderState = "CLOSE";
//        }

        targetX = x;
        double currentSliderPos = sliderLeft.getCurrentPosition();

        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentSliderPos, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();
    }

    //Motion Profiling
    public void goTo(double x, double maxVel, double maxAccel) {
        targetX = x;
        double currentx = sliderLeft.getCurrentPosition();
        motionProfilex = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentx, 0, 0),
                new MotionState(x, 0, 0),
                maxVel,
                maxAccel
        );
        timer.reset();
    }



    public double update() {
        if (motionProfilex != null) {
            MotionState xState = motionProfilex.get(timer.seconds());

            return xState.getX();

        }
        return 0;
    }


}
