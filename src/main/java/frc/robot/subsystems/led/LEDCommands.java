package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDCommands {
    private static LED led = LED.getInstance();

    public static Command setState(State s) {
        return new InstantCommand(
            () -> led.setState(s),
            led
        );
    }

    public static Command setIsAmping(boolean b) {
        return new InstantCommand(
            () -> led.setIsAmping(b),
            led
        );
    }

    public static Command togglePoliceMode() {
        return new InstantCommand(
            () -> led.togglePoliceModeEnabled(),
            led
        );
    }

    public static Command setPoliceMode(int mode) {
        return new InstantCommand(
            () -> led.setPoliceMode(mode),
            led
        );
    }
}