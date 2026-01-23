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

public class DaytonaOffseason extends SimBaseRobot {
    SwerveChassis driveTrain;
    Elevator elevator;
    Arm armRotator, intake;
    Flywheel armWheels, centerWheels, intakeWheels;
    Transform3d originalRobotToArmCarrier, originalRobotToArmRotator, originalRobotToArmWheels, originalRobotToIntake, originalRobotToCenterWheels, originalRobotToIntakeWheels;

    List<SimMechanism> mechanisms;

    public DaytonaOffseason(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Elevator Carrier (Elevator)
        elevator = new Elevator(this,
                new Transform3d(Meters.of(0.105), Meters.of(0), Meters.of(0.11), new Rotation3d()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getKrakenX60(2),
                5.6,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(1.35),
                Meters.of(0.0),
                2,
                false);

        // Arm Rotator (Arm)
        originalRobotToArmCarrier = new Transform3d(Meters.of(-0.07), Meters.of(0.005), Meters.of(0.17), new Rotation3d());
        armRotator = new Arm(this,
                new Transform3d(originalRobotToArmCarrier.getMeasureX(), originalRobotToArmCarrier.getMeasureY(), originalRobotToArmCarrier.getMeasureZ(), originalRobotToArmCarrier.getRotation()),
                new Rotation3d(1, 0, 0), // Arm rotations around this axis
                "arm_rotator",
                DCMotor.getKrakenX60(2), //MODIFY
                68.571428,
                1.0,
                Meters.of(1),
                Degrees.of(-999), // -999
                Degrees.of(999.0), // 999
                Degrees.of(0.0),
                false,
                false);


        // Arm Wheels (Flywheel)
        originalRobotToArmWheels = new Transform3d(Meters.of(-0.23), Meters.of(0), Meters.of(0.17), new Rotation3d());
        armWheels = new Flywheel(this,
                new Transform3d(originalRobotToArmWheels.getMeasureX(), originalRobotToArmWheels.getMeasureY(), originalRobotToArmWheels.getMeasureZ(), originalRobotToArmWheels.getRotation()),
                new Rotation3d(0, 1, 0), // Flywheel rotates around this axis
                "armWheels",
                DCMotor.getKrakenX60(1),
                2.25,
                0.01,
                false,
                true);



        // Intake (Arm)
        originalRobotToIntake = new Transform3d(Meters.of(-0.24), Meters.of(0.0), Meters.of(0.15), new Rotation3d());
        intake = new Arm(this,
                new Transform3d(originalRobotToIntake.getMeasureX(), originalRobotToIntake.getMeasureY(), originalRobotToIntake.getMeasureZ(), originalRobotToIntake.getRotation()),
                new Rotation3d(0, 1, 0), // Arm rotations around this axis
                "intake",
                DCMotor.getKrakenX60(1),
                86.495726,
                1.0,
                Meters.of(1),
                Degrees.of(-999), // -999
                Degrees.of(999.0), // 999
                Degrees.of(0.0),
                false,
                false);

        // Intake wheels to center (Flywheel)
        originalRobotToCenterWheels = new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0.0), new Rotation3d());
        centerWheels = new Flywheel(this,
                new Transform3d(originalRobotToCenterWheels.getMeasureX(), originalRobotToCenterWheels.getMeasureY(), originalRobotToCenterWheels.getMeasureZ(), originalRobotToCenterWheels.getRotation()),
                new Rotation3d(1, 0, 0), // Flywheel rotates around this axis
                "centerWheels",
                DCMotor.getKrakenX60(1),
                2.25,
                0.01,
                false,
                true);


        // Intake wheels  (Flywheel)
        originalRobotToIntakeWheels = new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), new Rotation3d());
        intakeWheels = new Flywheel(this,
                new Transform3d(originalRobotToIntakeWheels.getMeasureX(), originalRobotToIntakeWheels.getMeasureY(), originalRobotToIntakeWheels.getMeasureZ(), originalRobotToIntakeWheels.getRotation()),
                new Rotation3d(0, 1, 0), // Flywheel rotates around this axis
                "intakeWheels",
                DCMotor.getKrakenX60(1),
                2.25,
                0.01,
                false,
                true);

        // List of mechanisms
        mechanisms = List.of(elevator, armRotator, armWheels, intake, centerWheels, intakeWheels);
    }

    @Override
public void Update() {
    driveTrain.Update();
    mechanisms.forEach(mech -> mech.Update());

    // BRAZO
    // Actualizar la posición del brazo en base en el elevador
    Pose3d elevatorPose = elevator.GetPoses3d().get(0);
    armRotator.SetRobotToMechanism(
        originalRobotToArmCarrier.plus(new Transform3d(elevatorPose.getTranslation(), new Rotation3d()))
    );

    // ARM WHEELS
    // Update the arm wheels' position based on the arm's position
    double armRotatorAngle = armRotator.GetAngle();
    double armLength = 0.55; // Assuming this is the arm's length (r)

    // Convert polar to rectangular
    double armWheelsY = armLength * -Math.sin(armRotatorAngle); // x = r * cos(θ)
    double armWheelsZ = armLength * Math.cos(armRotatorAngle);  // y = r * sin(θ)

    // Update the wheels rotation based on the arm's rotator
    double armRotatorAngleX = armRotator.GetPoses3d().get(0).getRotation().getX(); // Angle in radians
    double armRotatorAngleY = armRotator.GetPoses3d().get(0).getRotation().getY(); // Angle in radians
    double armRotatorAngleZ = armRotator.GetPoses3d().get(0).getRotation().getZ(); // Angle in radians

    // Create new Pose3d for armWheels
    Pose3d armRotatorPose = new Pose3d(
        new Translation3d(0, armWheelsY, armWheelsZ + elevatorPose.getTranslation().getZ()), 
        new Rotation3d(armRotatorAngleX, armRotatorAngleY, armRotatorAngleZ)
    );

    // Update the Arm Wheels' position
    armWheels.SetRobotToMechanism(
        originalRobotToArmWheels.plus(new Transform3d(armRotatorPose.getTranslation(), armRotatorPose.getRotation()))
    );

    // CENTER WHEELS
    // Update the arm wheels' position based on the arm's position
    double intakeAngle = intake.GetAngle();
    double intakeLenght = 0.15; // Assuming this is the arm's length (r)

    // Convert polar to rectangular
    double intakeX = intakeLenght * Math.sin(intakeAngle); // x = r * cos(θ)
    double intakeZ = intakeLenght * Math.cos(intakeAngle);  // y = r * sin(θ)

    // Update the wheels rotation based on the arm's rotator
    double intakeAngleX = intake.GetPoses3d().get(0).getRotation().getX(); // Angle in radians
    double intakeAngleY = intake.GetPoses3d().get(0).getRotation().getY(); // Angle in radians
    double intakeAngleZ = intake.GetPoses3d().get(0).getRotation().getZ(); // Angle in radians

    // Create new Pose3d for armWheels
    Pose3d centerWheelsPose = new Pose3d(
        new Translation3d(intakeX - 0.24,0, intakeZ + 0.15
        ), 
        new Rotation3d(intakeAngleX, intakeAngleY, intakeAngleZ)
    );

    // Update the Arm Wheels' position
    centerWheels.SetRobotToMechanism(
        originalRobotToCenterWheels.plus(new Transform3d(centerWheelsPose.getTranslation(), centerWheelsPose.getRotation()))
    );





   // INTAKE WHEELS
    // Update the arm wheels' position based on the arm's position
    double mainIntakeAngle = intake.GetAngle();
    double mainIintakeLenght = 0.3; // Assuming this is the arm's length (r)

    // Convert polar to rectangular
    double mainIntakeX = mainIintakeLenght * Math.sin(mainIntakeAngle); // x = r * cos(θ)
    double mainIntakeZ = mainIintakeLenght * Math.cos(mainIntakeAngle);  // y = r * sin(θ)

    // Update the wheels rotation based on the arm's rotator
    double mainIntakeAngleX = intake.GetPoses3d().get(0).getRotation().getX(); // Angle in radians
    double mainIntakeAngleY = intake.GetPoses3d().get(0).getRotation().getY(); // Angle in radians
    double mainIntakeAngleZ = intake.GetPoses3d().get(0).getRotation().getZ(); // Angle in radians

    // Create new Pose3d for armWheels
    Pose3d intakeWheelsPose = new Pose3d(
        new Translation3d(mainIntakeX - 0.24,0, mainIntakeZ + 0.15
        ), 
        new Rotation3d(mainIntakeAngleX, mainIntakeAngleY, mainIntakeAngleZ)
    );

    // Update the Arm Wheels' position
    intakeWheels.SetRobotToMechanism(
        originalRobotToIntakeWheels.plus(new Transform3d(intakeWheelsPose.getTranslation(), intakeWheelsPose.getRotation()))
    );













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