package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class UpperPixelDrop extends SequentialCommandGroup {
    public UpperPixelDrop(Intake intake, Outtake outtake) {
//        Outtake.crab.setPosition(0.65);
//        Outtake.stopper.setPosition(0.3);
        super(
                new StopperCommand(outtake, Outtake.StopperState.OPEN)
        );
    }
}
