package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class CANClimberSubsystem extends SubsystemBase {
    private final SparkMax climber;

    private final RelativeEncoder climberEncoder;


    public CANClimberSubsystem() {
        climber = new SparkMax(Constants.ClimbConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);
        climberEncoder = climber.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climber.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }


    private double getPos(){
        return climberEncoder.getPosition() * Constants.ClimbConstants.GEAR_RATIO /Constants.DriveConstants.ENCODER_UNITS_PER_METER;
    }
    private void goTo(double target){

        double current = getPos();
        double error = target - current;

        double output = error * Constants.ClimbConstants.kP;

        output = Math.max(
                -Constants.ClimbConstants.MAX_OUTPUT,
                Math.min(Constants.ClimbConstants.MAX_OUTPUT, output)
        );

        climber.set(output);
    }

    public Command goToTop() {

        return run(() -> goTo(Constants.ClimbConstants.TOP_SETPOINT))
                .until(() -> Math.abs(getPos() - Constants.ClimbConstants.TOP_SETPOINT) < 0.01)
                .finallyDo(() -> climber.set(0));
    }

    public Command goToBottom() {

        return run(() -> goTo(Constants.ClimbConstants.BOTTOM_SETPOINT))
                .until(() -> Math.abs(getPos() - Constants.ClimbConstants.BOTTOM_SETPOINT) < 0.01)
                .finallyDo(() -> climber.set(0));
    }


    @Override
        public void periodic() {
            SmartDashboard.putNumber("climber", getPos());
    }
}
