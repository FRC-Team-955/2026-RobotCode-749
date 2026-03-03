package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;


public class CANClimberSubsystem extends SubsystemBase {
    private final SparkMax climber;

    private final RelativeEncoder climberEncoder;

    public double topEncoderValue;

    public CANClimberSubsystem() {
        climber = new SparkMax(Constants.ClimbConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);
        climberEncoder = climber.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climber.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void setClimberZero() {
        topEncoderValue = climberEncoder.getPosition() + 5; // the +5 is to allow us to be able to manually change the top point
    }

    public void stop() {
        climber.set(0);
    }

    public void goUp() {
        if (climberEncoder.getPosition() < topEncoderValue) {
            climber.set(4*MathUtil.clamp((topEncoderValue-climberEncoder.getPosition())/10, 0, 1));
        } else {
            climber.set(0);
        }
        // (topEncoderValue-climberEncoder.getPosition())/10 will divide the difference so whenever it gets within 10 encoder ticks it
        // will start to slow down because when divided by 10 it will give you a value less than 1 (same goes for below function)
    }

    public void goDown() {
        if (climberEncoder.getPosition() > topEncoderValue-Constants.ClimbConstants.ENCODER_CAP) { // ENCODER_CAP needs to be tuned
            climber.set(-4*Math.abs(MathUtil.clamp((topEncoderValue-climberEncoder.getPosition())/10, -1, 0)));
        } else {
            climber.set(0);
        }
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
        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        SmartDashboard.putNumber("Climber bottom point", topEncoderValue);
    }
}
