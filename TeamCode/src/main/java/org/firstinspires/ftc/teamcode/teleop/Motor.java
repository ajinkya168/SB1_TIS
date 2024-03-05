package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;

@TeleOp(group = "Trial")
public class Motor extends LinearOpMode {
//    private DcMotorEx motor_1;
    Hanger hanger;
    Servo dropLeft, dropRight;
    boolean a, b, x, y, up, down, left, right = false;

    public void runOpMode() {
//
        hanger = new Hanger(hardwareMap, telemetry);
//        Outtake outtake = new Outtake(hardwareMap, telemetry);
//        Slider slider = new Slider(hardwareMap, telemetry);
//        dropLeft = hardwareMap.get(Servo.class, "dropLeft");
//        dropRight = hardwareMap.get(Servo.class, "dropRight");
//        motor_1 = hardwareMap.get(DcMotorEx.class, "Motor");
//        motor_1.setDirection(DcMotor.Direction.FORWARD);
//        motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Status Initialized");
        telemetry.update();

//        Hanger.hangerLeft.setPosition(0.5);
//        Hanger.hangerRight.setPosition(0.5);
//        Outtake.outtakeWrist.setPosition(0.5);
//        dropLeft.setPosition(0);
//        dropRight.setPosition(0);
//        Outtake.setOuttakeArm(0.99);
//        Outtake.outtakeLeft.setPosition(0.5);
//        Outtake.outtakeRight.setPosition(0.5);
//0.9 to open
//        0.3 to close

        waitForStart();

     //open 0.5 lock 0.9
        //open 0.5 lock 0.1
        while (opModeIsActive()) {
            if (gamepad1.a && !a)
            {
////                Hanger.hangerMotor.setPower(1);
//
////                Hanger.hangerLeft.setPosition(Hanger.hangerLeft.getPosition()+ 0.1);
////                slider.sliderRight.setTargetPosition(100);
////                slider.sliderRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////                slider.sliderRight.setPower(0.8);
                Hanger.setHangerMotor(Hanger.hangerMotor.getCurrentPosition() + 100);
////                Slider.extendTo(Slider.sliderLeft.getCurrentPosition() + 100, 0.8);
//               a = true;
            }
            if(!gamepad1.a){
                a = false;
            }
            if (gamepad1.b && !b)
            {
////                Hanger.hangerLeft.setPosition(Hanger.hangerLeft.getPosition() - 0.1);
                Hanger.setHangerMotor(Hanger.hangerMotor.getCurrentPosition() - 100);
////                slider.sliderLeft.setTargetPosition(100);
////                slider.sliderLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////                slider.sliderLeft.setPower(0.8);
////                Slider.extendTo(Slider.sliderLeft.getCurrentPosition() - 100, 0.8);
//                b = true;
            }
            if(!gamepad1.b){
                b = false;
            }
////            if(gamepad1.x && !x){
////                Hanger.hangerRight.setPosition(Hanger.hangerRight.getPosition()+ 0.1);
////                x= true;
////            }
//////            left 0.2 and right 0.8
//////            drop 0.3
////            if(!gamepad1.x){
////                x= false;
////            }
////            if(gamepad1.y && !y){
////                Hanger.hangerRight.setPosition(Hanger.hangerRight.getPosition()- 0.1);
////                y = true;
////            }
////            if(!gamepad1.y){
////                y= false;
////            }
////            if(gamepad1.dpad_up && !up){
////                dropLeft.setPosition(dropLeft.getPosition() + 0.05);
////                up = true;
////            }
////            if(!gamepad1.dpad_up){
////                up= false;
////            }
////            if(gamepad1.dpad_down && !down){
////                dropLeft.setPosition(dropLeft.getPosition() - 0.05);
////                down = true;
////            }
////            if(!gamepad1.dpad_down){
////                down= false;
////            }
//            if(gamepad1.dpad_left && !left){
////                Hanger.hangerRight.setPosition(Hanger.hangerRight.getPosition() + 0.05);
//                Outtake.outtakeWrist.setPosition(Outtake.outtakeWrist.getPosition() + 0.1);
//                left = true;
//            }
//            if(!gamepad1.dpad_left){
//                left = false;
//            }
//            if(gamepad1.dpad_right && !right){
////                Hanger.hangerRight.setPosition(Hanger.hangerRight.getPosition() - 0.05);
//                Outtake.outtakeWrist.setPosition(Outtake.outtakeWrist.getPosition() - 0.1);
//                right = true;
//            }
//            if(!gamepad1.dpad_right){
//                right = false;
//            }
//
////            telemetry.addData("left", Hanger.hangerLeft.getPosition());
//            telemetry.addData("right" ,Hanger.hangerRight.getPosition());
////            telemetry.addData("dropLeft", dropLeft.getPosition());
//            telemetry.addData("dropRight", Outtake.outtakeWrist.getPosition());
            telemetry.addData("current", Hanger.hangerMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
//
        }

    }
}