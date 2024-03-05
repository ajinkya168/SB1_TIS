package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;

public class OuttakeWristCommand extends InstantCommand {

    public OuttakeWristCommand(Outtake outtake, Outtake.OuttakeWristState state){
        super(
                () -> outtake.update(state)
        );
    }
}
