package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Robot extends SampleMecanumDrive {


    public static double Kp_slider = 0.01;
    public static double Ki_slider = 0;
    public static double Kd_slider = 0;
    public static double Kf_slider = 0;

    private Telemetry telemetry;
    //    Turret robotTurret ;
    Slider robotSlider ;

    //    private PIDController turretController;
    private PIDController sliderController;


    public static double targetDegree = 0;
    public static double targetSlider = 0;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
//        robotTurret = turret;


//        turretController = new PIDController(Kp_turret, Ki_turret, Kd_turret);

        this.telemetry = telemetry;
    }


    @Override
    public void update() {
        super.update();
        // TURRET
//        turretController.setPID(Kp_turret, Ki_turret, Kd_turret);

//
//        double currentTurretServoDegree = robotTurret.getDegree();
//        double turretServoPID = turretController.calculate(currentTurretServoDegree, targetDegree);
//        double ff_turret = 1*Kf_turret;
//        double turretServoPower = ff_turret + turretServoPID;
//
//        robotTurret.set(turretServoPower, turretServoPower);
//
//        this.telemetry.addData("Servo Power", turretServoPower);
//        this.telemetry.addData("Turret Target", targetDegree);
//        this.telemetry.addData("Turret Current: ", currentTurretServoDegree);

        /// SLIDER
//        sliderController.setPID(Kp_slider, Ki_slider, Kd_slider);
//        double currentSliderPosition = robotSlider.getPosition();
//        double SliderPID = sliderController.calculate(currentSliderPosition, robotSlider.update());
//        double ff_Slider = 1*Kf_slider;
//        double sliderPower = ff_Slider + SliderPID;
//
//        sliderPower = Range.clip(sliderPower,-1,1);
//
//        robotSlider.set(sliderPower);

//        this.telemetry.addData("Slider Power", sliderPower);
//        this.telemetry.addData("Slider Target", targetSlider);
//        this.telemetry.addData("Slider Current: ", currentSliderPosition);
//        this.telemetry.addData("slider",robotSlider.getPosition());
//        this.telemetry.addData("Slider rightslider: ",robotSlider.rightSliderCurrentPos());
//        this.telemetry.addData("Slider leftslider ",robotSlider.getPosition());

    }
}
