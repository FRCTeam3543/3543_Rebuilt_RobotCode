package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int mode = 0;
    double offset = 0;

    public Lights() {
        led = new AddressableLED(Constants.LightsConstants.port);
        buffer = new AddressableLEDBuffer(Constants.LightsConstants.length);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < Constants.LightsConstants.C_length; i++) {
            setRGB(i, Constants.LightsConstants.Colors.WHITE);
        }
        var alliance = DriverStation.getAlliance();
        for (int i = 0; i < Constants.LightsConstants.four_length; i++) {
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Blue) {
                    setRGB(i + Constants.LightsConstants.C_length, Constants.LightsConstants.Colors.BLUE);
                } else {
                    setRGB(i + Constants.LightsConstants.C_length, Constants.LightsConstants.Colors.RED);
                }
            }
        }
        for (int i = 0; i < Constants.LightsConstants.length - Constants.LightsConstants.C4_length; i++) {
            if (mode == 0) {
                if (alliance.isPresent()) {
                    if (alliance.get() == Alliance.Blue) {
                        setRGB(i + Constants.LightsConstants.C4_length, Constants.LightsConstants.Colors.BLUE);
                    } else {
                        setRGB(i + Constants.LightsConstants.C4_length, Constants.LightsConstants.Colors.RED);
                    }
                }
            } else if (mode == 1) {
                setRGB(i + Constants.LightsConstants.C4_length, blend(Constants.LightsConstants.Colors.BLUE, Constants.LightsConstants.Colors.RED, i, Constants.LightsConstants.length - Constants.LightsConstants.C4_length));
            } else if (mode == 2) {
                setHSV(i + Constants.LightsConstants.C4_length, calcHSV(offset, i, Constants.LightsConstants.length - Constants.LightsConstants.C4_length));
                offset += 0.1;
                offset %= 180;
            }
        }
        sendBuffer();
    }

    public int[] blend(int[] color1, int[] color2, int index, int length) {
        double mult = (double)index / (length - 1);
        return new int[] {
            color1[0] + (int)Math.round(mult * (color2[0] - color1[0])), 
            color1[1] + (int)Math.round(mult * (color2[1] - color1[1])), 
            color1[2] + (int)Math.round(mult * (color2[2] - color1[2]))
        };
    }

    public int[] calcHSV(double offset, int index, int length) {
        double mult = (double)index / (length - 1);
        return new int[] {(int)(180 * mult + offset) % 180, 255, 255};
    }

    public void sendBuffer() {
        led.setData(buffer);
    }

    public void setRGB(int index, int[] color) {
        buffer.setRGB(index, color[0], color[1], color[2]);
    }

    public void setHSV(int index, int[] color) {
        buffer.setHSV(index, color[0], color[1], color[2]);
    }
}