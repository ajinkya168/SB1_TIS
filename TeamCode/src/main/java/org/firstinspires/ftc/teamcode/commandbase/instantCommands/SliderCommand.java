package org.firstinspires.ftc.teamcode.commandbase.instantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.commandbase.Subsystems.Slider;

public class SliderCommand extends InstantCommand {

    public SliderCommand(Slider slider, Slider.SliderState state){
        super(
                () -> slider.update(state)
        );
    }
}
