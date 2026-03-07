package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDSystem {
    AddressableLED m_led;


    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(192, 96);
    private final LEDPattern m_percent = LEDPattern.progressMaskLayer(()->RobotState.shooterRatio);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 0.27 meter per second.
    private final LEDPattern m_scrollingRainbow =
            m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.27), kLedSpacing);

    public LEDSystem(){
        m_led =  new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(30);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
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



}
