package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ClimbConstants.ENCODER_CAP;


public class CANClimberSubsystem extends SubsystemBase {
    private final SparkMax climber;

    private final RelativeEncoder climberEncoder;
    public final PIDController PIDController = new PIDController(Constants.ClimbConstants.KP, Constants.ClimbConstants.KI, Constants.ClimbConstants.KD);
    public double mode = 1;

    public double topEncoderValue;
    public double bottomEncoderValue;

    public CANClimberSubsystem() {
        climber = new SparkMax(Constants.ClimbConstants.CLIMBER_ID, SparkLowLevel.MotorType.kBrushless);
        climberEncoder = climber.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climber.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void changeMode() {
        mode ++;
        if (mode == 3) {
            mode = 1;
        }
    }

    public void setTopValue() {
        topEncoderValue = climberEncoder.getPosition();
        bottomEncoderValue = climberEncoder.getPosition()-ENCODER_CAP;
    }

    public void setBottomValue() {
        bottomEncoderValue = climberEncoder.getPosition();
        topEncoderValue = climberEncoder.getPosition()+ENCODER_CAP;
    }

    public void setClimberTopAsCurrent() {
        topEncoderValue = climberEncoder.getPosition(); // the +5 is to allow us to be able to manually change the top point
    }
    public void setClimberBottomAsCurrent() {
        topEncoderValue = climberEncoder.getPosition() + 5 - ENCODER_CAP; // the +5 is to allow us to be able to manually change the top point
    }

    public void stop() {
        climber.set(0);
    }

    public void goUp() {
        if (mode == 2) {
            PIDController.setSetpoint(topEncoderValue);
            climber.set(MathUtil.clamp(PIDController.calculate(climberEncoder.getPosition()), 0, 4));
        } else if (mode == 1) {
            if (climberEncoder.getPosition() < topEncoderValue) {//climberEncoder.getPosition() < topEncoderValue
                //climber.set(4*MathUtil.clamp((topEncoderValue-climberEncoder.getPosition())/10, 0, 1));
                climber.set(4);
            } else {
                climber.set(0);
            }
        }
        // (topEncoderValue-climberEncoder.getPosition())/10 will divide the difference so whenever it gets within 10 encoder ticks it
        // will start to slow down because when divided by 10 it will give you a value less than 1 (same goes for below function)
    }

    public void goDown() {
        if (mode == 2) {
            PIDController.setSetpoint(topEncoderValue - 50);
            climber.set(MathUtil.clamp(PIDController.calculate(climberEncoder.getPosition()), -4, 0));
        } else if (mode == 1) {
            if (climberEncoder.getPosition() > topEncoderValue - ENCODER_CAP) { // ENCODER_CAP needs to be tuned: climberEncoder.getPosition() > topEncoderValue-Constants.ClimbConstants.ENCODER_CAP
                //climber.set(-4*Math.abs(MathUtil.clamp((topEncoderValue-climberEncoder.getPosition())/10, -1, 0)));
                climber.set(-4);
            } else {
                climber.set(0);
            }
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
/*
    public Command goToTop() {

        return run(() -> goTo(Constants.ClimbConstants.TOP_SETPOINT))
                .until(() -> Math.abs(getPos() - Constants.ClimbConstants.TOP_SETPOINT) < 0.01)
                .finallyDo(() -> climber.set(0));
    }

    public Command goToBottom() {

        return run(() -> goTo(Constants.ClimbConstants.BOTTOM_SETPOINT))
                .until(() -> Math.abs(getPos() - Constants.ClimbConstants.BOTTOM_SETPOINT) < 0.01)
                .finallyDo(() -> climber.set(0));
    }*/


    @Override
        public void periodic() {
        SmartDashboard.putNumber("climber", getPos());
        SmartDashboard.putNumber("Climber Encoder", climberEncoder.getPosition());
        SmartDashboard.putNumber("Climber top point", topEncoderValue);
        if (mode == 1) {
            SmartDashboard.putString("Climber Mode", "Manual");
        } else if (mode == 2) {
            SmartDashboard.putString("Climber Mode", "Limited");
        }


        //System.out.print("CLIMBER ENC: "); System.out.println(climberEncoder.getPosition());
        // TOP -93
        // BOTTOM -207
    }
}
