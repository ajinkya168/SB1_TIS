package org.firstinspires.ftc.teamcode.commandbase.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake extends SubsystemBase {
    public static Servo outtakeWrist, stopper, outtakeRight,outtakeLeft,crab;

    // OUTTAKE ARM VALUES
    public static double outtakeArmInit = 0;
    public static double outtakeArmDrop = 0.95;

    // OUTTAKE WRIST VALUES
    public static double outtakeWristInit = 0;
    public static double outtakeWristDrop = 0.7;

    // OUTTAKE STOPPER VALUES
    public static double close = 0.5;
    public static double open = 0.3;

    // OUTTAKE CRAB VALUES
    public static double neutral = 0.5;
    public static double grab = 0.65;
    public static double grab_init = 0.7;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) {
        outtakeWrist = hardwareMap.get(Servo.class, "BedTilt");
        stopper = hardwareMap.get(Servo.class, "Stopper");
        outtakeRight = hardwareMap.get(Servo.class, "RightAxon");
        outtakeLeft = hardwareMap.get(Servo.class, "LeftAxon");
        crab = hardwareMap.get(Servo.class, "Crab");
    }

    public enum OuttakeArmState{
        INIT,
        DROP
    }

    public enum OuttakeWristState{
        INIT,
        DROP
    }

    public enum StopperState{
        CLOSE,
        OPEN
    }

    public enum CrabState{
        NEUTRAL,
        GRAB,
        GRAB_INIT
    }

    OuttakeArmState outtakeArmState = OuttakeArmState.INIT;
    OuttakeWristState outtakeWristState = OuttakeWristState.INIT;
    StopperState stopperState = StopperState.CLOSE;
    CrabState crabState = CrabState.NEUTRAL;

    public void update(OuttakeArmState state){
        outtakeArmState = state;
        switch (state){
            case INIT:
                setOuttakeArm(outtakeArmInit);
                break;
            case DROP:
                setOuttakeArm(outtakeArmDrop);
                break;
        }
    }

    public void update(OuttakeWristState state){
        outtakeWristState = state;
        switch (state){
            case INIT:
                setServo(outtakeWrist, outtakeWristInit);
                break;
            case DROP:
                setServo(outtakeWrist, outtakeWristDrop);
                break;
        }
    }

    public void update(StopperState state){
        stopperState = state;
        switch (state){
            case CLOSE:
                setServo(stopper, close);
                break;
            case OPEN:
                setServo(stopper, open);
                break;
        }
    }

    public void update(CrabState state){
        crabState = state;
        switch (state){
            case NEUTRAL:
                setServo(crab, neutral);
                break;
            case GRAB:
                setServo(crab, grab);
                break;
            case GRAB_INIT:
                setServo(crab, grab_init);
                break;
        }
    }

    public static void setOuttakeArm(double AxonPos){
        outtakeRight.setPosition(1-AxonPos);
        outtakeLeft.setPosition(AxonPos);
    }

    public static void setServo(Servo servo, double pos){
        servo.setPosition(pos);
    }

    public static void SetOuttakeServoOnePosition(double outtakeWristPos) throws InterruptedException {
        outtakeWrist.setPosition(outtakeWristPos);
    }
    public static void SetOuttakeServoTwoPosition(double stopperPos) throws InterruptedException {
        stopper.setPosition(stopperPos);
    }
    public static void SetOuttakeServoThreePosition(double outtakeRightPos) throws InterruptedException {
        outtakeRight.setPosition(outtakeRightPos);
    }
    public static void SetOuttakeServoFourPosition(double outtakeLeftPos) throws InterruptedException {
        outtakeLeft.setPosition(outtakeLeftPos);
    }
    public static void SetOuttakeServoFivePosition(double crabPos) throws InterruptedException {
        crab.setPosition(crabPos);
    }
    public static void SetOuttakeServos(double outakeServoPos,double AxonPos) throws InterruptedException {

        outtakeWrist.setPosition(outakeServoPos);
        stopper.setPosition(outakeServoPos);
        outtakeRight.setPosition(AxonPos);
        outtakeLeft.setPosition(AxonPos);
        crab.setPosition(outakeServoPos);
    }

}
