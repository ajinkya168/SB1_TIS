package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Trial")
public class CurrentPrint extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx sliderR = hardwareMap.get(DcMotorEx.class, "sliderMotorOne");
        DcMotorEx sliderL = hardwareMap.get(DcMotorEx.class, "sliderMotorTwo");

        sliderR.setDirection(DcMotorEx.Direction.REVERSE);
        sliderL.setDirection(DcMotorEx.Direction.REVERSE);

        sliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                sliderL.setPower(1);
                sleep(1000);
                sliderL.setPower(0);
            }
            if(gamepad1.b){
                sliderR.setPower(1);
                sleep(1000);
                sliderR.setPower(0);
            }

            telemetry.addData("current", sliderL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("current", sliderR.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
