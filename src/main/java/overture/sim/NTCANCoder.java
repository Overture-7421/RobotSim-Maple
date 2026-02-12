package overture.sim;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;

public class NTCANCoder {
    public static class Config {
        public String Name;

        public Supplier<Angle> EncoderPosition;
        public Supplier<AngularVelocity> EncoderSpeed;

        public Boolean Inverted;
    }

    private DoublePublisher encoderSpeedEntry, encoderPositionEntry;
    private Config config;
    private Double invMultiplier = 1.0;

    public NTCANCoder(Config config) {
        this.config = config;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable motorTable = ntInst.getTable(config.Name);

        encoderSpeedEntry = motorTable.getDoubleTopic("cancoder_speed").publish();
        encoderPositionEntry = motorTable.getDoubleTopic("cancoder_position").publish();

        encoderSpeedEntry.set(0);
        encoderPositionEntry.set(0);

        if (config.Inverted) {
            invMultiplier = -1.0;
        }
    }

    public void Update() {
        encoderPositionEntry.set(config.EncoderPosition.get().in(Rotations) * invMultiplier);
        encoderSpeedEntry.set(config.EncoderSpeed.get().in(RotationsPerSecond) * invMultiplier);
    }
}
