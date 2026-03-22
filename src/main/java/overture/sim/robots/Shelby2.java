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
    Flywheel intakeFlywheel, indexerFlywheel;

    Transform3d originalRobotToIntakeFlywheel, originalRobotToIndexerFlywheel;


    List<SimMechanism> mechanisms;

    public Shelby2(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivetrain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Intake
        intake = new Elevator(this,
                new Transform3d(Meters.of(0.03), Meters.of(-0.09), Meters.of(0.13), new Rotation3d()),
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

        // Intake FlyWheels
        originalRobotToIntakeFlywheel = new Transform3d(Meters.of(0.47), Meters.of(0.0), Meters.of(0.2), new Rotation3d());
        intakeFlywheel = new Flywheel(this,
                new Transform3d(originalRobotToIntakeFlywheel.getMeasureX(), originalRobotToIntakeFlywheel.getMeasureY(), originalRobotToIntakeFlywheel.getMeasureZ(), originalRobotToIntakeFlywheel.getRotation()),
                new Rotation3d(0, 1, 0), 
                "intake_flywheel",
                DCMotor.getKrakenX60(1),
                1,
                0.01,
                false,
                true);

        // Indexer FlyWheels
        originalRobotToIndexerFlywheel = new Transform3d(Meters.of(0.09), Meters.of(0.0), Meters.of(0.12), new Rotation3d());
        indexerFlywheel = new Flywheel(this,
                new Transform3d(originalRobotToIndexerFlywheel.getMeasureX(), originalRobotToIndexerFlywheel.getMeasureY(), originalRobotToIndexerFlywheel.getMeasureZ(), originalRobotToIndexerFlywheel.getRotation()),
                new Rotation3d(0, 1, 0), 
                "indexer_flywheel",
                DCMotor.getKrakenX60(1),
                1,
                0.01,
                false,
                true);


                
        // List of mechanisms
        mechanisms = List.of(intake, intakeFlywheel, indexerFlywheel);
    }

        // ---------------------------------------
        // OFFSET FIJO DE INTAKE A ROLLERS
        // ---------------------------------------
        private static final Transform3d intakeToRollers =
    new Transform3d(
        Meters.of(0.445), // offset en X
        Meters.of(0.1), // offset en Y
        Meters.of(0.075), // offset en Z
        new Rotation3d()
    );
        // ---------------------------------------
        // OFFSET FIJO DE INTAKE A ROLLERS
        // ---------------------------------------
    
    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());


        // ---------------------------------------
        // INTAKE & INTAKE ROLLERS
        // ---------------------------------------
        Transform3d robotToIntake = intake.GetPoses3d().get(0).minus(new Pose3d());
        Transform3d robotToRollers = robotToIntake.plus(intakeToRollers);
        intakeFlywheel.SetRobotToMechanism(robotToRollers);
        // ---------------------------------------
        // INTAKE & INTAKE ROLLERS
        // ---------------------------------------
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