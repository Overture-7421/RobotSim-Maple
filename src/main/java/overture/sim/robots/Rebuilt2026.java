package overture.sim.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.mechanisms.arm.Arm;
import overture.sim.mechanisms.elevator.Elevator;
import overture.sim.mechanisms.flywheel.Flywheel;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class Rebuilt2026 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Flywheel spindexer;
    Arm intake;
    Transform3d originalRobotToSpindexer, originalRobotToIntake;

    List<SimMechanism> mechanisms;

    public Rebuilt2026(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Spindexer (Flywheel)
        originalRobotToSpindexer = new Transform3d(Meters.of(-0.1), Meters.of(0), Meters.of(0.15), new Rotation3d());
        spindexer = new Flywheel(this,
                new Transform3d(originalRobotToSpindexer.getMeasureX(), originalRobotToSpindexer.getMeasureY(), originalRobotToSpindexer.getMeasureZ(), originalRobotToSpindexer.getRotation()),
                new Rotation3d(0, 0, 1), // Flywheel rotates around this axis
                "Spindexer",
                DCMotor.getKrakenX60(1),
                0.33,
                0.01,
                false,
                true);

        // Intake (Arm)
        originalRobotToIntake = new Transform3d(Meters.of(0.0), Meters.of(0.0), Meters.of(0.5), new Rotation3d());
        intake = new Arm(this,
                new Transform3d(originalRobotToIntake.getMeasureX(), originalRobotToIntake.getMeasureY(), originalRobotToIntake.getMeasureZ(), originalRobotToIntake.getRotation()),
                new Rotation3d(0, 1, 0), // Arm rotations around this axis
                "intake",
                DCMotor.getKrakenX60(1),
                1,
                1.0,
                Meters.of(1),
                Degrees.of(-999),
                Degrees.of(999.0),
                Degrees.of(0.0),
                false,
                false);

        // List of mechanisms
        mechanisms = List.of(spindexer, intake);
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