package frc.robot.subsystems;
//TODO random filler deelte this later
public class Shooter {
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }

    public boolean getHoming() {
        return false;
    }
}
