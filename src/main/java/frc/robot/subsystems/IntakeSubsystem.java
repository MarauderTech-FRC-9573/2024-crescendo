package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.MutableMeasure;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.RunCommand;

//Almost same code as the shootersubsystem.
public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_IntakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax m_ArmMotor = new CANSparkMax(IntakeConstants.ArmMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  Boolean intakeMotor;
  double positionArm;
  
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  public PIDController pid = new PIDController(IntakeConstants.kPArm, IntakeConstants.kIArm, IntakeConstants.kDArm);
  private final SimpleMotorFeedforward m_shooterFeedforward =
  new SimpleMotorFeedforward(
  IntakeConstants.kSVolts,
  IntakeConstants.kVVoltSecondsPerRotation,
  IntakeConstants.kAVoltSecondsSquaredPerRotation);
  
  
  
  
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
    m_ArmMotor.setVoltage(volts.in(Volts));
  },
  log -> {
    log.motor("arm-motor")
    .voltage(m_appliedVoltage.mut_replace(m_ArmMotor.get() * RobotController.getBatteryVoltage(), Volts))
    .linearPosition(m_distance.mut_replace(m_ArmMotor.getEncoder().getPosition(), Meters))
    .linearVelocity(m_velocity.mut_replace(m_ArmMotor.getEncoder().getVelocity(), MetersPerSecond));
  }, this));
  
  
  
  public IntakeSubsystem() {
    
  }
  
  public void setIntakeMotor(double speed) {
    m_IntakeMotor.set(speed);
  }
  
  public double getArmPostition() {
    return positionArm;
  }
  
  public void setArmMotor(double speed) {
    m_ArmMotor.set(speed);
  }
  
  public void setArmPosition(double position) {
    m_ArmMotor.getEncoder().setPosition(0.0);
  }
  
  public void stop() {
    m_IntakeMotor.set(0);
    m_ArmMotor.set(0);
  }

  @Override
  public void periodic() { 
    positionArm = m_ArmMotor.getEncoder().getPosition();
    System.out.println("Arm Motor Position: " + positionArm);
  }

  // SYSID Functions
  public Command runArm(DoubleSupplier runSpeed) {
    return new RunCommand(() -> {
      m_ArmMotor.setVoltage(pid.calculate(m_ArmMotor.getEncoder().getVelocity(), runSpeed.getAsDouble()) + m_shooterFeedforward.calculate(runSpeed.getAsDouble()));
    });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { 
    return m_sysIdRoutine.quasistatic(direction);
  } 

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}