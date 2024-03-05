package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.IntakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class DropPos extends SequentialCommandGroup {
    public DropPos(Intake intake, Outtake outtake) {
//Outtake.setOuttakeArm(0.95);
//        Outtake.outtakeWrist.setPosition(0.7);
//        Outtake.stopper.setPosition(0.5);
//        Outtake.crab.setPosition(0.7);
//        Intake.SetIntakePosition(0.55);
        super(
                new ParallelCommandGroup(
                    new OuttakeWristCommand(outtake, Outtake.OuttakeWristState.DROP),
                    new StopperCommand(outtake, Outtake.StopperState.CLOSE),
                    new CrabCommand(outtake, Outtake.CrabState.GRAB_INIT),
                    new IntakeWristCommand(intake, Intake.IntakeWristState.INTAKE_UP)
                )
        );
    }
}

