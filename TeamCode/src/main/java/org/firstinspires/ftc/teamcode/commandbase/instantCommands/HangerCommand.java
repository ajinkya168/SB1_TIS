package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Hanger;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;

public class HangerCommand extends InstantCommand {

    public HangerCommand(Hanger hanger, Hanger.HangerState state){
        super(
                () -> hanger.update(state)
        );
    }
}
