
package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.swerveCANcoderConfig;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANCoder absolute encoders.
 */

public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private CANCoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;

    public int angleCounter = 0;
    public int positionCounter = 0;
    public int setDesireCounter = 0;
    
    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

        /* Angle Encoder Config */

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configEncoders();


        // lastAngle = getState().angle;
    }

    private void configEncoders()
    {
        // absolute encoder
        angleEncoder.configAllSettings(swerveCANcoderConfig);

        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);


    //    relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);


        relAngleEncoder = mAngleMotor.getEncoder();
    //    relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);


        synchronizeEncoders();
    //    mDriveMotor.burnFlash();
    //    mAngleMotor.burnFlash();

    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkMaxPIDController controller = mAngleMotor.getPIDController();

        // set pid profile in slot 0 for speed control
        controller.setP(Constants.Swerve.angleKP, 0);
        controller.setI(Constants.Swerve.angleKI,0);
        controller.setD(Constants.Swerve.angleKD,0);
        controller.setFF(Constants.Swerve.angleKFF,0);
        controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        mAngleMotor.setClosedLoopRampRate(Constants.Swerve.angleRampRate);

        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_v, 0);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_v, 0);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_v, 0);

        //Set pid profile in slot 1 for position control
        controller.setP(Constants.Swerve.angleKP, 1);
        controller.setI(Constants.Swerve.angleKI,1);
        controller.setD(Constants.Swerve.angleKD,1);
        controller.setFF(Constants.Swerve.angleKFF,1);

        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 1);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_p, 1);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_p, 1);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_p, 1);

    }

    private void configDriveMotor()
    {
        mDriveMotor.restoreFactoryDefaults();
        SparkMaxPIDController controller = mDriveMotor.getPIDController();
        controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);

        controller.setP(Constants.Swerve.driveKP_v,0);
        controller.setI(Constants.Swerve.driveKI,0);
        controller.setD(Constants.Swerve.driveKD,0);
        controller.setFF(Constants.Swerve.driveKFF,0);
        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_v, 0);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_v, 0);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_v, 0);
    
        // set pid profile in slot 1 for position control
        controller.setP(Constants.Swerve.driveKP_p,1);
        controller.setI(Constants.Swerve.driveKI,1);
        controller.setD(Constants.Swerve.driveKD,1);
        controller.setFF(Constants.Swerve.driveKFF,1);
        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 1);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_p, 1);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_p, 1);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_p, 1);

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        SmartDashboard.putNumber("setDesireState",setDesireCounter++);
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState,false);//isOpenLoop);

        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkMaxPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, CANSparkMax.ControlType.kSmartVelocity, 0);
        SmartDashboard.putNumber("Speed",velocity);
        SmartDashboard.putNumber("output",mDriveMotor.getAppliedOutput());

    }

    public void setAngle(SwerveModuleState desiredState)
    {
       // if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
       // {
       //     mAngleMotor.stopMotor();
       //     return;
       // }
        Rotation2d angle = desiredState.angle;
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        SparkMaxPIDController controller = mAngleMotor.getPIDController();

        double degReference = angle.getDegrees();
        //controller.setReference (degReference, ControlType.kSmartMotion, 1);
        controller.setReference (degReference/15, ControlType.kSmartMotion, 1);
        SmartDashboard.putNumber("Angle Counter",angleCounter++);
    }

    public void setPosition(double position)
    {
        SparkMaxPIDController controller = mDriveMotor.getPIDController();
        double encoderDelta = position / Constants.Swerve.driveRevToMeters;
        double currentPosition = mDriveMotor.getEncoder().getPosition();
        controller.setReference (currentPosition + encoderDelta, ControlType.kSmartMotion,1);
        SmartDashboard.putNumber("SetPosition",encoderDelta);
     //   Translation2d e;
    }
    private void setYaw()
    {

    }
    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {

        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    public void synchronizeEncoders()
    {

        double absolutePosition =getCanCoder().getDegrees() - angleOffset.getDegrees();
    //    relAngleEncoder.setPosition(absolutePosition);
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega()
    {
        return angleEncoder.getVelocity()/360;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }
}
