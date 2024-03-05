package org.firstinspires.ftc.teamcode.geometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.drivetrain.MecanumDrivetrain;

public class RobotHardwareK {
    //TODO DRIVE
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    //TODO ROBOT SETUP
    private static RobotHardwareK instance = null;  // ref variable to use robot hardware
    public boolean enabled;                          //boolean to return instance if robot is enabled.

    private HardwareMap hardwareMap;
    public MecanumDrivetrain drivetrain;


    public static RobotHardwareK getInstance() {
        if (instance == null) {
            instance = new RobotHardwareK();
        }
        instance.enabled = true;
        return instance;
    }


    //TODO HARDWAREMAP

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        drivetrain = new MecanumDrivetrain();

        //Drive
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void write() {
        drivetrain.write();
    }

    public void write( double slow) {
        drivetrain.write(slow);
    }




}
