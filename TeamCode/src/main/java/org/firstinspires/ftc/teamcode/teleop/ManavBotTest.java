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
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Trial")
@Config
public class ManavBotTest extends LinearOpMode {
    public static int acc=500;
    public static int vel=1000;
    Slider slider=null;
    @Override
    public void runOpMode() throws InterruptedException {
        slider= new Slider(hardwareMap, telemetry);
        Robot drive = new Robot(hardwareMap, telemetry);


        waitForStart();
        while (opModeIsActive()){

            //drivetrain ---------------------------------------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3),
                    Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3));

            drive.setWeightedDrivePower(
                    new Pose2d(input.getX() , input.getY() , -gamepad1.right_stick_x )
            );


            if(gamepad1.a){
                slider.goTo(100);
            }
            if(gamepad1.b){
                slider.goTo(0);
            }
            if (gamepad1.x){
                slider.goTo(80,vel,acc);

            }

            drive.update();
            slider.update();


            telemetry.addData("current L", Slider.sliderLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("current R", Slider.sliderRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("pos L", Slider.sliderLeft.getCurrentPosition());
            telemetry.addData("pos R", Slider.sliderRight.getCurrentPosition());
            telemetry.update();
        }

    }
}



