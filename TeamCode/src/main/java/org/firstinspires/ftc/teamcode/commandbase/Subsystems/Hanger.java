package org.firstinspires.ftc.teamcode.commandbase.Subsystems;

import android.nfc.cardemulation.HostNfcFService;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanger extends SubsystemBase {
    public static DcMotorEx hangerMotor;
    public static Servo hangerRight, hangerLeft, hangerServo;
    public static double motorPower = 0.8;
    public static double extendPosition = 1.0;
    public static double down_pos1 = 0.9, down_pos2= 0.13;
    public static double hang_pos1 = 0.3, hang_pos2 = 0;

    public static int hang_count = 6000;
    public static int down_count = -3000;
    public Hanger(HardwareMap hardwareMap, Telemetry telemetry) {

        hangerRight = hardwareMap.get(Servo.class, "hangerRight");
        hangerLeft = hardwareMap.get(Servo.class, "hangerLeft");

        hangerMotor = hardwareMap.get(DcMotorEx.class, "hangerMotor");
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hangerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public enum HangerState{
        INIT,
        HANG_SERVO,
        DOWN_SERVO,
        HANG_MOTOR,
        DOWN_MOTOR
    }

        HangerState hangerState = HangerState.INIT;

    public void update(HangerState state){
        hangerState = state;
        switch (state){
            case INIT:
                break;
            case HANG_SERVO:
                setHangerServo(hang_pos1, hang_pos2);
                break;
            case DOWN_SERVO:
                setHangerServo(down_pos1, down_pos2);
                break;
            case HANG_MOTOR:
                setHangerMotor(hang_count);
                break;
            case DOWN_MOTOR:
                setHangerMotor(down_count);
                break;
        }
    }

    public static void setHangerServo(double hangerPos1, double hangerPos2){
//        hangerRight.setPosition(1 - hangerPos);
        hangerLeft.setPosition(hangerPos1);
        hangerRight.setPosition(hangerPos2);
    }
    public static void setHangerMotor(int pos){
        hangerMotor.setTargetPosition(pos);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(motorPower);

    }
    public static void PutDownRobot(int pos){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - pos );
//        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 4000 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void HangerINC(int pos){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + pos );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void HangerDEC(int pos){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - pos );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
//    public static void StopHanger(){
//        hangerMotor.setPower(0);
//    }
}
