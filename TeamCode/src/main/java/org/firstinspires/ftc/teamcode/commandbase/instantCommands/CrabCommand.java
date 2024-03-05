package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;

public class CrabCommand extends InstantCommand {

    public CrabCommand(Outtake outtake, Outtake.CrabState state){
        super(
                () -> outtake.update(state)
        );
    }
}
