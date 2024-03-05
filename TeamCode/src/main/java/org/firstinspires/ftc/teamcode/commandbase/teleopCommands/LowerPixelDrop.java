package org.firstinspires.ftc.teamcode.commandbase.teleopCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.CrabCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.IntakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.SliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class LowerPixelDrop  extends SequentialCommandGroup {
    public LowerPixelDrop(Intake intake, Outtake outtake) {
//        Outtake.crab.setPosition(0.65);
//        Outtake.stopper.setPosition(0.3);
//        sleep(1000);
//        Outtake.stopper.setPosition(0.5);
//        Outtake.crab.setPosition(0.5);
        super(
                new CrabCommand(outtake, Outtake.CrabState.GRAB),
                new StopperCommand(outtake, Outtake.StopperState.OPEN),
                new WaitCommand(500),
                new StopperCommand(outtake, Outtake.StopperState.CLOSE),
                new CrabCommand(outtake, Outtake.CrabState.NEUTRAL)
        );
    }
}
