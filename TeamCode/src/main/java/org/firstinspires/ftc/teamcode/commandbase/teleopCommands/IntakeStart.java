package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.IntakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class IntakeStart extends SequentialCommandGroup {
    public IntakeStart(Intake intake, Outtake outtake) {
//        Intake.SetIntakePosition(intakeServoPos);
//        Intake.IntakeStart();
//        Outtake.crab.setPosition(0.5);
        super(
                new ParallelCommandGroup(
                        new IntakeWristCommand(intake, Intake.IntakeWristState.INTAKE_DOWN),
                        new CrabCommand(outtake, Outtake.CrabState.NEUTRAL)
                )
        );
    }
}

