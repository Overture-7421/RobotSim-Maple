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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Rebuilt2026 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Flywheel spindexer, intakeRollers, shooterWheels;
    Arm intake, turret, hood;
    Elevator elevator;
    Transform3d originalRobotToSpindexer, originalRobotToIntake, originalRobotToElevator, originalRobotToIntakeRollers, originalRobotToTurret, originalRobotToHood, originalRobotToShooterWheels;
    private DoublePublisher encoderSpeedEntryCanCoder1, encoderPositionEntryCanCoder1, encoderSpeedEntryCanCoder2, encoderPositionEntryCanCoder2;
    final double turretRatio = 41.666; 


    List<SimMechanism> mechanisms;

    public Rebuilt2026(String name, Pose2d startingPose) {
        super(name, startingPose);

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable motorTable = ntInst.getTable(name + "/cancoders/");


        encoderSpeedEntryCanCoder1 = motorTable.getDoubleTopic("turretcancoder1/cancoder_speed").publish();
        encoderPositionEntryCanCoder1 = motorTable.getDoubleTopic("turretcancoder1/cancoder_position").publish();

        encoderSpeedEntryCanCoder2 = motorTable.getDoubleTopic("turretcancoder2/cancoder_speed").publish();
        encoderPositionEntryCanCoder2 = motorTable.getDoubleTopic("turretcancoder2/cancoder_position").publish();


            encoderSpeedEntryCanCoder1.set(0);
            encoderPositionEntryCanCoder1.set(0);

            encoderSpeedEntryCanCoder2.set(0);
            encoderPositionEntryCanCoder2.set(0);


        
        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Spindexer (Flywheel)
        originalRobotToSpindexer = new Transform3d(Meters.of(0.0), Meters.of(0), Meters.of(0.14), new Rotation3d());
        spindexer = new Flywheel(this,
                new Transform3d(originalRobotToSpindexer.getMeasureX(), originalRobotToSpindexer.getMeasureY(), originalRobotToSpindexer.getMeasureZ(), originalRobotToSpindexer.getRotation()),
                new Rotation3d(0, 0, 1), // Flywheel rotates around this axis
                "spindexer",
                DCMotor.getKrakenX60(1),
                0.33,
                0.01,
                false,
                true);

        // Intake (Arm)
        originalRobotToIntake = new Transform3d(Meters.of(-0.19), Meters.of(0.0), Meters.of(0.2), new Rotation3d());
        intake = new Arm(this,
                new Transform3d(originalRobotToIntake.getMeasureX(), originalRobotToIntake.getMeasureY(), originalRobotToIntake.getMeasureZ(), originalRobotToIntake.getRotation()),
                new Rotation3d(0, 1, 0), // Arm rotations around this axis
                "intake",
                DCMotor.getKrakenX60(1),
                50,
                0.1,
                Meters.of(1),
                Degrees.of(-9999),
                Degrees.of(9999.0),
                Degrees.of(0.0),
                false,
                false);

        // Elevator Carrier (Elevator)
        originalRobotToElevator = new Transform3d(Meters.of(0.17), Meters.of(0.293), Meters.of(0.17), new Rotation3d());
        elevator = new Elevator(this,
                new Transform3d(originalRobotToElevator.getMeasureX(), originalRobotToElevator.getMeasureY(), originalRobotToElevator.getMeasureZ(), originalRobotToElevator.getRotation()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getKrakenX60(2),
                12,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(0.33),
                Meters.of(0.0),
                1,
                false);

        // Intake Rollers (Flywheel)
        originalRobotToIntakeRollers = new Transform3d(Meters.of(0.15), Meters.of(0.0), Meters.of(0.175), new Rotation3d());
        intakeRollers = new Flywheel(this,
                new Transform3d(originalRobotToIntakeRollers.getMeasureX(), originalRobotToIntakeRollers.getMeasureY(), originalRobotToIntakeRollers.getMeasureZ(), originalRobotToIntakeRollers.getRotation()),
                new Rotation3d(0, 1, 0), // Flywheel rotates around this axis
                "intakeRollers",
                DCMotor.getKrakenX60(1),
                1,
                0.01,
                false,
                true);

        // Turret (Arm)
        originalRobotToTurret = new Transform3d(Meters.of(0.26), Meters.of(-0.11), Meters.of(0.28), new Rotation3d());
        turret = new Arm(this,
                new Transform3d(originalRobotToTurret.getMeasureX(), originalRobotToTurret.getMeasureY(), originalRobotToTurret.getMeasureZ(), originalRobotToTurret.getRotation()),
                new Rotation3d(0, 0, 1), // Arm rotations around this axis
                "turret",
                DCMotor.getKrakenX60(1),
                turretRatio,
                0.01,
                Meters.of(1),
                Degrees.of(-9999),
                Degrees.of(9999.0),
                Degrees.of(0.0),
                false,
                false);
        
        // Hood (Arm)
        originalRobotToHood = new Transform3d(Meters.of(-0.145), Meters.of(-0.0), Meters.of(0.07), new Rotation3d());
        hood = new Arm(this,
                new Transform3d(originalRobotToHood.getMeasureX(), originalRobotToHood.getMeasureY(), originalRobotToHood.getMeasureZ(), originalRobotToHood.getRotation()),
                new Rotation3d(0, 1, 0), // Arm rotations around this axis
                "hood",
                DCMotor.getKrakenX60(1),
                142.4,
                0.1,
                Meters.of(1),
                Degrees.of(-9999),
                Degrees.of(9999.0),
                Degrees.of(0.0),
                false,
                false);

        // Shooter Wheels (Flywheel)
        originalRobotToShooterWheels = new Transform3d(Meters.of(0.13), Meters.of(-0.11), Meters.of(0.35), new Rotation3d());
        shooterWheels = new Flywheel(this,
                new Transform3d(originalRobotToShooterWheels.getMeasureX(), originalRobotToShooterWheels.getMeasureY(), originalRobotToShooterWheels.getMeasureZ(), originalRobotToShooterWheels.getRotation()),
                new Rotation3d(0, 1, 0), // Flywheel rotates around this axis
                "shooterWheels",
                DCMotor.getKrakenX60(1),
                1,
                0.01,
                false,
                true);
        
        // List of mechanisms
        mechanisms = List.of(spindexer, intake, elevator, intakeRollers, turret, hood, shooterWheels);
    }

    // OFFSET FIJO DESDE EL INTAKE HASTA LOS ROLLERS
private static final Transform3d intakeToRollers =
    new Transform3d(
        Meters.of(-0.16), // distancia a lo largo del brazo
        Meters.of(0.0),
        Meters.of(0.22),
        new Rotation3d()
    );

    // OFFSET FIJO DESDE LA TORRETA AL HOOD
private static final Transform3d turretToHood =
    new Transform3d(
        Meters.of(-0.145),
        Meters.of(0.0),
        Meters.of(0.07),
        new Rotation3d()
    );

    // OFFSET FIJO DESDE LA TORRETA A LAS SHOOTER WHEELS
private static final Transform3d turretToShooterWheels =
    new Transform3d(
        Meters.of(-0.13),
        Meters.of(0.0),
        Meters.of(0.07),
        new Rotation3d()
    );


    
    @Override
public void Update() {
    driveTrain.Update();
    mechanisms.forEach(mech -> mech.Update());

        // ---------------------------------------
        // INTAKE & INTAKE ROLLERS
        // ---------------------------------------
        Transform3d robotToIntake = intake.GetPoses3d().get(0).minus(new Pose3d());
        Transform3d robotToRollers = robotToIntake.plus(intakeToRollers);
        intakeRollers.SetRobotToMechanism(robotToRollers);
        // ---------------------------------------
        // INTAKE & INTAKE ROLLERS
        // ---------------------------------------

        // ---------------------------------------
        // TURRET ROTATION HERITAGE TO HOOD AND SHOOTER WHEELS
        // ---------------------------------------
        Transform3d robotToTurret =
            turret.GetPoses3d().get(0).minus(new Pose3d());

        Transform3d robotToHood =
            robotToTurret.plus(turretToHood);
        hood.SetRobotToMechanism(robotToHood);

        Transform3d robotToShooter =
            robotToTurret.plus(turretToShooterWheels);
        shooterWheels.SetRobotToMechanism(robotToShooter);
        // ---------------------------------------
        // TURRET ROTATION HERITAGE TO HOOD AND SHOOTER WHEELS
        // ---------------------------------------

        // ---------------------------------------
        // TURRET CANCODERS //
        // ---------------------------------------

        // Obtener la posición del motor de la torreta (en este caso, el ángulo de la torreta)
        double turretAngleC = turret.GetAngle() * (turretRatio / (2.0 * Math.PI)); // Convertir a grados
        // Tener un valor de cancoder1, con base al angulo de la torreta y el gear ratio
        double cancoder1Value = turretAngleC * 0.08571918395;
        //Publicar
        encoderPositionEntryCanCoder1.set(cancoder1Value);


        // TURRET CANCODER 2 //
        // Tener un valor de cancoder1, con base al angulo de la torreta y el gear ratio
        double cancoder2Value = turretAngleC * 0.09230797633;
        // Publicar
        encoderPositionEntryCanCoder2.set(cancoder2Value);

        // ---------------------------------------
        // TURRET CANCODERS //
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