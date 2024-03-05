package org.firstinspires.ftc.teamcode.commandbase.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drone extends SubsystemBase {
    public static Servo droneServo;
    public static double shootPosition = 0.9, initialPosition = 0.6;

    public enum DroneState{
        INIT,
        SHOOT
    }
    DroneState droneState = DroneState.INIT;

    public Drone(HardwareMap hardwareMap, Telemetry telemetry) {
        droneServo = hardwareMap.get(Servo.class, "droneServo");
    }

    public void update(DroneState state){
        droneState = state;
        switch (state){
            case INIT:
                droneServo.setPosition(initialPosition);
                break;
            case SHOOT:
                droneServo.setPosition(shootPosition);
                break;
        }
    }

    public static void shootDrone(){
        droneServo.setPosition(shootPosition);
    }
    public static void initialPos(){
        droneServo.setPosition(initialPosition);
    }
    public static void inc(){
        droneServo.setPosition(droneServo.getPosition() + 0.1);
    }
    public static void dec(){
        droneServo.setPosition(droneServo.getPosition() - 0.1);
    }
}
