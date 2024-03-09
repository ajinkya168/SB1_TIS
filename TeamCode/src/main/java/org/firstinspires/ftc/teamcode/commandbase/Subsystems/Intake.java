package org.firstinspires.ftc.teamcode.commandbase.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    public static Servo  intakeLeft, intakeRight;
    public static DcMotorEx Intake;
    public static double intakePower = 1;
    public static double intakeArmServoPos = 0.0, intakeRightPos = 0.0, intakeLeftPos = 0.0;
    public static double intakeWristInitPos = 0.2;
    public static double intakeWristIntakeDownPos = 0.60;
    public static double getIntakeWristIntakeUpPos = 0.55;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeLeft = hardwareMap.get(Servo.class, "LeftIntake");
        intakeRight = hardwareMap.get(Servo.class, "RightIntake");
//        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public enum IntakeWristState{
        INIT,
        INTAKE_DOWN,
        INTAKE_UP
    }

    IntakeWristState intakeWristState = IntakeWristState.INIT;

    public void update(IntakeWristState state) {
        intakeWristState = state;
        switch (state){
            case INIT:
                SetIntakePosition(intakeWristInitPos);
                break;
            case INTAKE_DOWN:
                SetIntakePosition(intakeWristIntakeDownPos);
                break;
            case INTAKE_UP:
                SetIntakePosition(getIntakeWristIntakeUpPos);
                break;
        }
    }

    public static void SetIntakePosition(double intakeServoPos) {
        intakeRight.setPosition(intakeServoPos);
        intakeLeft.setPosition(1- intakeServoPos);
    }
//    public static void SetIntakeArmPosition(double intakeArmServoPos) throws InterruptedException {
//        intakeArmServo.setPosition(intakeArmServoPos);
//    }
    public static void IntakeStart()
    {
        Intake.setPower(intakePower);
    }
    public static void IntakeStop()
    {
        Intake.setPower(0.0);
    }
    public static void IntakeReverse()
    {
        Intake.setPower(-intakePower);
    }
}
