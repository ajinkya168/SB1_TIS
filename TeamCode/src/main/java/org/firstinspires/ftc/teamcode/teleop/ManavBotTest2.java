package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
public class ManavBotTest2 extends LinearOpMode {
Intake intake;
boolean a= false;
    @Override
    public void runOpMode() throws InterruptedException {
     intake = new Intake(hardwareMap, telemetry);
        Intake.intakeRight.setPosition(0.2);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.b){
                Intake.IntakeStart();
            }
            if(gamepad1.x){
                Intake.IntakeStop();
            }
            //0.45
            //5.90551
            if(gamepad1.a && !a){
                Intake.intakeRight.setPosition(Intake.intakeRight.getPosition() + 0.05);
                a = true;
            }
            if(!gamepad1.a){
                a = false;
            }
        telemetry.addData("servo", Intake.intakeRight.getPosition());
          telemetry.update();
        }
    }


}



