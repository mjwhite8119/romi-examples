package frc.robot.sensors;

import frc.robot.utilities.MatchedColor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * ColorMatcher takes RGB input and outputs a color.
 */
public class ColorSensor {
    private NetworkTable m_colorData;
    
    public ColorSensor(NetworkTable colorData) {
        m_colorData = colorData;
    }

    public MatchedColor getMatchedColor() {
        
        NetworkTableEntry blueEntry = m_colorData.getEntry("Blue");
        NetworkTableEntry greenEntry = m_colorData.getEntry("Green");
        NetworkTableEntry redEntry = m_colorData.getEntry("Red");

        if (redEntry.getDouble(0.0) > 8000.0) {
          return MatchedColor.RED;
        } 
        else if (greenEntry.getDouble(0.0) > 8000.0) {
          return MatchedColor.YELLOW;
        } 
        else if (blueEntry.getDouble(0.0) > 4300.0 & greenEntry.getDouble(0.0) > 4900.0) {
          return MatchedColor.BLUE;
        } 
        return MatchedColor.UNKNOWN;
    }
}
