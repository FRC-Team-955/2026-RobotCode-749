package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDSystem extends SubsystemBase {
    Timer timer = new Timer();
    AddressableLED m_led;


    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 96);
    private final LEDPattern m_percent = LEDPattern.progressMaskLayer(()->RobotState.shooterRatio);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 0.27 meter per second.
    private final LEDPattern m_scrollingRainbow =
            m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.27), kLedSpacing);
    private final LEDPattern pink = LEDPattern.solid(Color.kDeepPink);
    private final LEDPattern orange = LEDPattern.solid(Color.kOrange);
    private final LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private final LEDPattern red = LEDPattern.solid(Color.kRed);
    double phase;
    double alliance = 0; // 1 is blue, 2 is red
    double autoWinner = 0; // 1 is blue, 2 is red

    public LEDSystem(){
        m_led =  new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(30);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void startTimer() {
        timer.reset();
        timer.start();
    }

    public void stopTimer() {
        timer.stop();
        timer.reset();
    }

    public void pink() {
        pink.applyTo(m_ledBuffer);
    }

    public void orange() {
        orange.applyTo(m_ledBuffer);
    }

    public void blue() {
        blue.applyTo(m_ledBuffer);
    }

    public void red() {
        red.applyTo(m_ledBuffer);
    }

    public void rainbow() {
        m_scrollingRainbow.applyTo(m_ledBuffer);
    }

    public void setLEDs() {
        m_led.setData(m_ledBuffer);
    }

    public void shooterLeds(){
        if(RobotState.shooterRatio >= 1|| RobotState.shooterRatio <0) {
            m_scrollingRainbow.applyTo(m_ledBuffer);
            // Set the LEDs
            m_led.setData(m_ledBuffer);
        }
        else{

            m_percent.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }
    }

    public void setAutoWinner() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            alliance = 2;
        } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            alliance = 1;
        }
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                autoWinner = 2;
            } else if (character == 'B') {
               autoWinner = 1;
            }
        }
    }

    public void setPhase() {
        if (DriverStation.getMatchTime() < 140.0) {
            phase = 6;
            if (DriverStation.getMatchTime() < 110.0) {
                phase = 5;
                if (DriverStation.getMatchTime() < 85.0) {
                    phase = 4;
                    if (DriverStation.getMatchTime() < 60.0) {
                        phase = 3;
                        if (DriverStation.getMatchTime() < 35.0) {
                            phase = 2;
                            if (DriverStation.getMatchTime() < 10) {
                                phase = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    public void redLEDS() {
        if (phase == 1 || phase == 6) {
            red();
        } else {
            hubLEDS();
        }
    }

    public void blueLEDS() {
        if (phase == 1 || phase == 6) {
            blue();
        } else {
            hubLEDS();
        }
    }

    public void hubLEDS() {
        if (phase == 2 || phase == 4) {
            if (autoWinner == 2) {
                blueLEDS();
            } else if (autoWinner == 1) {
                redLEDS();
            }
        } else if (phase == 3 || phase == 5) {
            if (autoWinner == 2) {
                redLEDS();
            } else if (autoWinner == 1) {
                blueLEDS();
            }
        }
    }

    @Override
    public void periodic() {
        setPhase();

        if (alliance == 0) {
            if (timer.get() == 0) {
                rainbow();
            } else if (timer.get() < 10.0) {
                double x = Math.round(10.0 * (10 - timer.get())) /10.0;
                SmartDashboard.putNumber("Time Until Hub Change", x);
                if (x < 6.0) {
                    if (x < 3.0) {
                        if (Math.round(MathUtil.inputModulus(4 * (10 - x), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    } else {
                        if (Math.round(MathUtil.inputModulus(2 * (10 - x), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    }
                } else {
                    pink();
                }
            } else if (timer.get() < 110.0) {
                double y = Math.round(10.0 * (25 - MathUtil.inputModulus(timer.get() - 10, 0, 25))) /10.0;
                SmartDashboard.putNumber("Time Until Hub Change", y);
                if (y < 6.0) {
                    if (y < 3.0) {
                        if (Math.round(MathUtil.inputModulus(4 * (25 - y), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    } else {
                        if (Math.round(MathUtil.inputModulus(2 * (25 - y), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    }
                } else {
                    pink();
                }
            } else if (timer.get() < 140.0) {
                double z = Math.round(10.0 * (30 - MathUtil.inputModulus(timer.get() - 110, 0, 30))) /10.0;
                SmartDashboard.putNumber("Time Until Hub Change", z);
                if (z < 6.0) {
                    if (z < 3.0) {
                        if (Math.round(MathUtil.inputModulus(4 * (30 - z), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    } else {
                        if (Math.round(MathUtil.inputModulus(2 * (30 - z), 0, 1)) == 0) {
                            pink();
                        } else {
                            orange();
                        }
                    }
                } else {
                    pink();
                }
            } else {
                pink();
            }
        } else if (alliance == 2) {
            redLEDS();
        } else if (alliance == 1) {
            blueLEDS();
        }

        setLEDs();
    }



}
