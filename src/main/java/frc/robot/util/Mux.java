package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Mux {
  private final int address;
  private final I2C i2c;

  /**
   * Constructs the multiplexer with a custom address
   * 
   * @param address The address of the bus
   */
  public Mux(int address) {
    this.address = address;
    i2c = new I2C(Port.kOnboard, this.address);
  }

  /**
   * Constructs the multiplexer with a default address
   */
  public Mux() {
    // Default address for the common PCB available
    this(0x70);
  }

  /**
   * Read list of enabled buses from the device.
   *
   * @return bit field of enabled buses
   */
  public int enabledBuses() {
    byte[] result = new byte[1];
    i2c.readOnly(result, 1);
    return result[0];
  }

  /**
   * Set the list of enabled buses
   *
   * @param buses list of buses to enable
   */
  public void setEnabledBuses(int... buses) {
    int writeValue = 0;
    for (int b : buses) {
      if (b >= availableBuses() || b < 0) {
        DriverStation.reportError("Invalid bus enabled on I2C Mux: " + b, true);
      } else {
        writeValue |= 1 << b;
      }
    }
    i2c.write(address, writeValue);
  }

  /**
   * Number of available buses
   *
   * @return number of available buses
   */  
  public int availableBuses() {
    return 8;
  }
}