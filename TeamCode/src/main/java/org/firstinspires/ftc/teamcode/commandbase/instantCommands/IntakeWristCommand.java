package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;

public class IntakeWristCommand extends InstantCommand {

    public IntakeWristCommand(Intake intake, Intake.IntakeWristState state){
        super(
                () -> intake.update(state)
        );
    }
}
