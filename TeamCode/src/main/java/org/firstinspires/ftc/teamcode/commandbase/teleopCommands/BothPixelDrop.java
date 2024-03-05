package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class BothPixelDrop extends SequentialCommandGroup {
    public BothPixelDrop(Intake intake, Outtake outtake) {

        super(
                new ParallelCommandGroup(
                        new CrabCommand(outtake, Outtake.CrabState.NEUTRAL),
                        new StopperCommand(outtake, Outtake.StopperState.OPEN)
                )
        );
    }
}
