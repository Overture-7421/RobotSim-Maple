package overture.sim.mechanisms.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import overture.sim.NTCANCoder;
import overture.sim.NTMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.robots.SimBaseRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulates an arm mechanism
 */
public class Arm extends SimMechanism {
	private NTMotor motor;
	private NTCANCoder cancoder;
	private SingleJointedArmSim singleJointedArmSim;
	private Rotation3d rotationAxis;

	/**
	 * Creates a new Arm
	 * 
	 * @param robot             The robot that this arm is attached to
	 * @param robotToArm        The transform from the robot to the arm
	 * @param rotationAxis      The axis of rotation of the arm
	 * @param name              The name of the arm
	 * @param gearbox           The gearbox of the arm
	 * @param gearing           The gearing of the arm
	 * @param jMomentofIntertia The moment of inertia of the arm
	 * @param armLength         The length of the arm
	 * @param miAngle           The minimum angle of the arm
	 * @param maxAngle          The maximum angle of the arm
	 * @param startingAngle     The starting angle of the arm
	 * @param inverted          If the motor is inverted
	 */
	public Arm(SimBaseRobot robot,
			Transform3d robotToArm,
			Rotation3d rotationAxis,
			String name,
			DCMotor gearbox,
			double gearing,
			double jMomentofIntertia,
			Distance armLength,
			Angle minAngle,
			Angle maxAngle,
			Angle startingAngle,
			boolean gravity,
			boolean inverted) {
		super(new Transform3d(robotToArm.getTranslation(), robotToArm.getRotation()));
		this.rotationAxis = rotationAxis;
		singleJointedArmSim = new SingleJointedArmSim(gearbox, gearing, jMomentofIntertia, armLength.in(Meters),
				minAngle.in(Radians), maxAngle.in(Radians), gravity, startingAngle.in(Radians));

		motor = new NTMotor(new NTMotor.Config() {
			{
				Name = robot.GetName() + "/motors/" + name;
				VoltageApplied = (voltage) -> singleJointedArmSim.setInputVoltage(voltage.in(Volts));
				EncoderPosition = () -> Radians.of(singleJointedArmSim.getAngleRads()).times(gearing);
				EncoderSpeed = () -> RadiansPerSecond.of(singleJointedArmSim.getVelocityRadPerSec()).times(gearing);
				Current = () -> Amps.of(singleJointedArmSim.getCurrentDrawAmps());
				Inverted = inverted;
			}
		});

		cancoder = new NTCANCoder(new NTCANCoder.Config() {
			{
				Name = robot.GetName() + "/cancoders/" + name;
				EncoderPosition = () -> Radians.of(singleJointedArmSim.getAngleRads());
				EncoderSpeed = () -> RadiansPerSecond.of(singleJointedArmSim.getVelocityRadPerSec());
				Inverted = inverted;
			}
		});
	}

	public double GetAngle() {
		return singleJointedArmSim.getAngleRads();
	}

	public double GetAngularVelocity() {
		return singleJointedArmSim.getVelocityRadPerSec();
	}

	@Override
	public void Update() {
		singleJointedArmSim.update(GetTimeStep());
		motor.Update();
		cancoder.Update();
	}

	@Override
	public List<Pose3d> GetPoses3d() {
		// Calculate the rotation based on the state of singleJointedArmSim and the
		// given rotation axis
		Rotation3d rotation = new Rotation3d(
				rotationAxis.getX() * singleJointedArmSim.getAngleRads(),
				rotationAxis.getY() * singleJointedArmSim.getAngleRads(),
				rotationAxis.getZ() * singleJointedArmSim.getAngleRads());

		return List.of(new Pose3d().transformBy(GetRobotToMechanism())
				.transformBy(new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), rotation)));
	}
}
