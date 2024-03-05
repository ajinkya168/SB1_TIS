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
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.OuttakeWristCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.SliderCommand;
import org.firstinspires.ftc.teamcode.commandbase.instantCommands.StopperCommand;

public class Back2Pos extends SequentialCommandGroup {
    public Back2Pos(Intake intake, Outtake outtake, Slider slider) {
//        slider.extendTo(levelZero, output_power);
//        sleep(500);
//        Outtake.setOuttakeArm(0);
//        Outtake.outtakeWrist.setPosition(0);
//        Outtake.stopper.setPosition(0.5);
//        Outtake.crab.setPosition(0.5);
//        Intake.SetIntakePosition(0.68);
        super(
//                new GripCommand(intake, intakeSubsystem.GripperState.PULL),
                new SliderCommand(slider, Slider.SliderState.LEVEL_ZERO),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new OuttakeArmCommand(outtake, Outtake.OuttakeArmState.INIT),
                        new OuttakeArmCommand(outtake, Outtake.OuttakeArmState.INIT),
                        new StopperCommand(outtake, Outtake.StopperState.CLOSE),
                        new CrabCommand(outtake, Outtake.CrabState.NEUTRAL),
                        new IntakeWristCommand(intake, Intake.IntakeWristState.INTAKE_DOWN)
                )
        );
    }
}

