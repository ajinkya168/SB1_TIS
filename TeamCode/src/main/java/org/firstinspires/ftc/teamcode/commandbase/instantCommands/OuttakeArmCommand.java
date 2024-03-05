package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;

public class OuttakeArmCommand extends InstantCommand {

    public OuttakeArmCommand(Outtake outtake, Outtake.OuttakeArmState state){
        super(
                () -> outtake.update(state)
        );
    }
}
