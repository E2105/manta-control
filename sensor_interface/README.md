# Pressure Interface

This node reads all sensors from the Bar30 Pressure sensor from Blue Robotics.

| Sensor readings | Command |
| :---            | :---    |
| Pressure        |         |
| Temperature     |         |

## Dependencies
* [Adafruit Python Library](https://github.com/adafruit/Adafruit_Python_BNO055) for BNO055.
* [Xsens Driver](https://github.com/ethz-asl/ethzasl_xsens_driver) for MTi30
* Python SMBus library for MS5837:

```
sudo apt install python-smbus
```