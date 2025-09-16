package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class FlashLights extends SequentialCommandGroup{

    private double startTime;
    
    public FlashLights() {

        LEDPattern off = LEDPattern.solid(Color.kBlack);
        LEDPattern on = LEDPattern.solid(Color.kBlue);

        addCommands(
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(off);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(on);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(off);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(on);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(off);}),
            new WaitCommand(0.1),
            new InstantCommand(() -> {Robot.lights.setBackLEDColor(on);})
        );
    }

}
