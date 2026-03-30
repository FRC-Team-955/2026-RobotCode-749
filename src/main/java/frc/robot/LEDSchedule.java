package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


///  Replacement to LEDSystem using delayed execution instead of continual updats. Does not yet do the rumble/flash things
public class LEDSchedule {
    private static final ScheduledExecutorService scheduler =
            Executors.newSingleThreadScheduledExecutor();


    public static void runAfterDelay(double seconds, Runnable task) {
        scheduler.schedule(task, (long)(seconds * 1000), TimeUnit.MILLISECONDS);
    }

    AddressableLED m_led;


    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer;

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 96);

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


    int alliance = 0; // 1 is blue, 2 is red. Jin why on earth did you use a double???????????????????????????????
    int autoWinner = 0; // 1 is blue, 2 is red

    public LEDSchedule(int alliance){
        this.alliance = alliance;
        m_led =  new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(30);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
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

    public void setAutoWinner(int winner){
        autoWinner = winner;
    }
    public void setAlliance(int alliance){
        this.alliance = alliance;
    }

    private void auto(){
        pink();
        setLEDs();
    }
    private void transition(){
        if(alliance==1){
            blue();
            setLEDs();
        }
        if(alliance==2){
            red();
            setLEDs();
        }
    }
    private void shiftA(){
        if(alliance==1 && autoWinner==1){
            blue();
        }
        else if(alliance==2 && autoWinner==2){
            red();
        }
        else{
            orange();
        }
        setLEDs();

    }
    private void shiftB(){
        if(alliance==1 && autoWinner==2){
            blue();
        }
        else if(alliance==2 && autoWinner==1){
            red();
        }
        else{
            orange();
        }
        setLEDs();
    }
    private void endgame(){
        pink();
        setLEDs();
    }
    private void stop(){
        //nothing...
    }
    public void goAuto(){
        auto(); //does this need a new thread? only do so if rainbow?
    }
    public void goTeleop(){
        transition();
        runAfterDelay(10, this::shiftA);  // after length of transition
        runAfterDelay(25+10, this::shiftB);
        runAfterDelay(25+25+10, this::shiftA);
        runAfterDelay(25+25+25+10, this::shiftB);
        runAfterDelay(25+25+25+25+10, this::endgame);
        runAfterDelay(30+25+25+25+25+10, this::stop);
    }
}
