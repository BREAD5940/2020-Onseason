package statespacesimulation.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.sim.EncoderSim;
import edu.wpi.first.hal.sim.SimDeviceSim;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.sim.SimDifferentialDrivetrain;
import edu.wpi.first.wpilibj.simulation.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import statespacesimulation.Constants;

public class DriveSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.
    private final SpeedControllerGroup m_leftMotors =
            new SpeedControllerGroup(new PWMVictorSPX(Constants.DriveConstants.kLeftMotor1Port),
                    new PWMVictorSPX(Constants.DriveConstants.kLeftMotor2Port));

    // The motors on the right side of the drive.
    private final SpeedControllerGroup m_rightMotors =
            new SpeedControllerGroup(new PWMVictorSPX(Constants.DriveConstants.kRightMotor1Port),
                    new PWMVictorSPX(Constants.DriveConstants.kRightMotor2Port));

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The left-side drive encoder
    private final Encoder m_leftEncoder =
            new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0],
                    Constants.DriveConstants.kLeftEncoderPorts[1],
                    Constants.DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
            new Encoder(Constants.DriveConstants.kRightEncoderPorts[0],
                    Constants.DriveConstants.kRightEncoderPorts[1],
                    Constants.DriveConstants.kRightEncoderReversed);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    // These classes help us simulate our drivetrain
    public SimDifferentialDrivetrain m_drivetrainSimulator;
    private EncoderSim m_leftEncoderSim;
    private EncoderSim m_rightEncoderSim;
    // The Field2d class simulates the field in the sim GUI. Note that we can have only one
    // instance!
    private Field2d m_fieldSim;
    private SimDouble m_gyroAngleSim;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Sets the distance per pulse for the encoders
        m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        if (RobotBase.isSimulation()) { // If our robot is simulated
            // This class simulates our drivetrain's motion around the field.
            m_drivetrainSimulator = new SimDifferentialDrivetrain(
                    Constants.DriveConstants.kDrivetrainPlant,
                    Constants.DriveConstants.kDriveKinematics,
                    Constants.DriveConstants.kLeftGearbox,
                    Constants.DriveConstants.kLeftGearing,
                    Constants.DriveConstants.kRightGearbox,
                    Constants.DriveConstants.kRightGearing);

            // The encoder and gyro angle sims let us set simulated sensor readings
            m_leftEncoderSim = new EncoderSim(m_leftEncoder.getFPGAIndex());
            m_rightEncoderSim = new EncoderSim(m_rightEncoder.getFPGAIndex());
            m_gyroAngleSim =
                    new SimDeviceSim("ADXRS450_Gyro" + "[" + SPI.Port.kOnboardCS0.value + "]")
                            .getDouble("Angle");

            // the Field2d class lets us visualize our robot in the simulation GUI.
            m_fieldSim = new Field2d();
        }
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        m_drivetrainSimulator.setInputs(m_leftMotors.get() * RobotController.getBatteryVoltage(),
                -m_rightMotors.get() * RobotController.getBatteryVoltage());
        m_drivetrainSimulator.update(0.020);

        m_leftEncoderSim.setDistance(m_drivetrainSimulator.getState(SimDifferentialDrivetrain.State.kLeftPosition));
        m_leftEncoderSim.setRate(m_drivetrainSimulator.getState(SimDifferentialDrivetrain.State.kLeftVelocity));
        m_rightEncoderSim.setDistance(m_drivetrainSimulator.getState(SimDifferentialDrivetrain.State.kRightPosition));
        m_rightEncoderSim.setRate(m_drivetrainSimulator.getState(SimDifferentialDrivetrain.State.kRightVelocity));
        m_gyroAngleSim.set(-m_drivetrainSimulator.getHeading().getDegrees());

        m_fieldSim.setRobotPose(getPose());
    }

    /**
     * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY!
     * If you want it to work elsewhere, use the code in
     * {@link SimDifferentialDrivetrain#getCurrentDrawAmps()}
     *
     * @return The drawn current in Amps.
     */
    public double getDrawnCurrentAmps() {
        return m_drivetrainSimulator.getCurrentDrawAmps();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts))
                > batteryVoltage) {
            leftVolts *= batteryVoltage / 12.0;
            rightVolts *= batteryVoltage / 12.0;
        }
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
        m_drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

}
