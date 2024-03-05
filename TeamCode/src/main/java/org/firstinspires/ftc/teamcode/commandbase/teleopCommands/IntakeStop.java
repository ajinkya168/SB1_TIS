package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.IntakeWristCommand;

public class IntakeStop extends SequentialCommandGroup {
    public IntakeStop(Intake intake, Outtake outtake) {
//        Intake.IntakeStop();
//            Intake.SetIntakePosition(0.55);
//            Outtake.crab.setPosition(0.65);
        super(
                new ParallelCommandGroup(
                        new IntakeWristCommand(intake, Intake.IntakeWristState.INTAKE_UP),
                        new CrabCommand(outtake, Outtake.CrabState.GRAB)
                )
        );
    }
}

