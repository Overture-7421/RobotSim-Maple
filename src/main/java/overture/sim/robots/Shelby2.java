package overture.sim.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import overture.sim.NTCANCoder;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.mechanisms.arm.Arm;
import overture.sim.mechanisms.elevator.Elevator;
import overture.sim.mechanisms.flywheel.Flywheel;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class Shelby2 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Elevator intake;

    List<SimMechanism> mechanisms;

    public Shelby2(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivetrain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Intake
        intake = new Elevator(this,
                new Transform3d(Meters.of(0.3), Meters.of(-0.09), Meters.of(0.13), new Rotation3d()),
                new Translation3d(1, 0, 0),
                "intake",
                DCMotor.getKrakenX60(2),
                1,
                Kilograms.of(0.01),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(0.31),
                Meters.of(0.0),
                1,
                false);

        // List of mechanisms
        mechanisms = List.of(intake);

    }

    
    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());
    }

    @Override
    public AbstractDriveTrainSimulation GetDriveTrain() {
        return driveTrain;
    }

    @Override
    public List<SimMechanism> GetMechanisms() {
        return mechanisms;
    }
}