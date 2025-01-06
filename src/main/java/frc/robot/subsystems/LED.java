package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

public class LED implements Subsystem {
    // LED Definitions
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public final LEDCommand Commands = new LEDCommand();

    // Patterns
    @SuppressWarnings("unused")
    private LEDPattern m_RedBlueCycle;

    // Rainbow
    private static final Distance kLedSpacing = Feet.of(3 / 72.0);
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
    private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    // Funny Cycle
    public int lastI = 0;

    public LED() {
        // PWM port 9
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(72);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void LEDSOff() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kBlack);
        }
        m_led.setData(m_ledBuffer);
    }

    public void applyColorCycle(int LedSpacing, Color Color1, Color Color2) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int j = i;
            j += lastI;
            if (j % LedSpacing == 0) {
                m_ledBuffer.setLED(i, Color1);
            } else {
                m_ledBuffer.setLED(i, Color2);
            }
        }
        lastI++;
        if (lastI == LedSpacing) {
            lastI = 0;
        }
        m_led.setData(m_ledBuffer);
    }

    public void twoColorCycle(int LedSpacing, Color Color1, Color Color2, int cycles, int timePerMoveMs) {
        for (int cycle = 0; cycle < cycles; cycle++) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                int j = i + lastI;
                if (j % LedSpacing == 0 || j % LedSpacing == 1 || j % LedSpacing == 2) {
                    m_ledBuffer.setLED(i, Color1);
                } else {
                    m_ledBuffer.setLED(i, Color2);
                }
            }
            lastI++;
            if (lastI >= LedSpacing) {
                lastI = 0;
            }
            m_led.setData(m_ledBuffer);
            try {
                Thread.sleep(timePerMoveMs);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void applyColorCycleV2(Color Color, Color Color1, Color Color2, int Spacing) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int j = i;
            j += lastI;
            if ((j % (Spacing)) == 0) {
                m_ledBuffer.setLED(i, Color1);
            } else if ((j % (Spacing)) == 1) {
                m_ledBuffer.setLED(i, Color);
            } else {
                m_ledBuffer.setLED(i, Color2);
            }
        }
        lastI++;
        if (lastI == Spacing) {
            lastI = 0;
        }
        m_led.setData(m_ledBuffer);
    }

    public void ShootingStar(Color color, int starLength, int speed, int cycles, boolean reversed) {
        if (reversed) {
            applyShootingStartReversed(starLength, speed, cycles);
        } else {
            applyShootingStar(starLength, speed, cycles);
        }
    }

    public void applyShootingStar(int starLength, int speed, int cycles) {
        Color starColor = new Color(0,0,255);
        Color backgroundColor = new Color(0, 0, 0);

        for (int cycle = 0; cycle < cycles; cycle++) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                for (int j = 0; j < m_ledBuffer.getLength(); j++) {
                    m_ledBuffer.setLED(j, backgroundColor);
                }
                for (int j = 0; j < starLength; j++) {
                    int position = i - j;
                    if (position >= 0 && position < m_ledBuffer.getLength()) {
                        m_ledBuffer.setLED(position, starColor);
                    }
                }
                m_led.setData(m_ledBuffer);
                try {
                    Thread.sleep(speed);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        LEDSOff();
    }

    public void applyShootingStartReversed(int starLength, int speed, int cycles) {
        Color starColor = new Color(0, 0, 125);
        Color backgroundColor = new Color(0, 0, 0);

        for (int cycle = 0; cycle < cycles; cycle++) {
            for (int i = m_ledBuffer.getLength() - 1; i >= 0; i--) {
                for (int j = 0; j < m_ledBuffer.getLength(); j++) {
                    m_ledBuffer.setLED(j, backgroundColor);
                }
                for (int j = 0; j < starLength; j++) {
                    int position = i + j;
                    if (position >= 0 && position < m_ledBuffer.getLength()) {
                        m_ledBuffer.setLED(position, starColor);
                    }
                }
                m_led.setData(m_ledBuffer);
                try {
                    Thread.sleep(speed);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        LEDSOff();
    }

    public void applyRainbow() {
        m_scrollingRainbow.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void closeLED() {
        m_led.stop();
    }

    public class LEDCommand {

        public Command applyColorCycle(int LedSpacing, Color Color1, Color Color2) {
            return run(() -> applyColorCycle(LedSpacing, Color1, Color2));
        }

        public Command test(int LedSpacing, Color Color1, Color Color2, int cycles, int time) {
            return runOnce(() -> twoColorCycle(LedSpacing, Color1, Color2, cycles, time));
        }

        public Command 
        shoot() {
            return runOnce(() -> twoColorCycle(10, Color.kSeaGreen, Color.kBlack, 144, 25));
        }

        public Command test2() {
            return runOnce(() -> ShootingStar(Color.kBlue, 2, 2, 1, false))
            .andThen(() -> ShootingStar(Color.kBlue, 2, 2, 1, true));
        }

        public Command off() {
            return runOnce(() -> LEDSOff());
        }
    }
}
