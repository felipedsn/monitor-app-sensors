# The MIT License (MIT)
#
# Copyright (c) 2016 Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import struct
import RPi.GPIO as GPIO
import time

# Minimal constants carried over from Arduino library
ADXL345_ADDRESS           = 0x53
ADXL345_REG_DEVID         = 0x00 # Device ID
ADXL345_REG_DATAX0        = 0x32
ADXL345_REG_DATAY0        = 0x34
ADXL345_REG_DATAZ0        = 0x36

ADXL345_REG_POWER_CTL     = 0x2D # Power-saving features control
ADXL345_REG_DATA_FORMAT   = 0x31
ADXL345_REG_BW_RATE       = 0x2C

ADXL345_REG_THRESH_TAP    = 0xD1
ADXL345_REG_DUR           = 0x21
ADXL345_REG_LATENT        = 0x22
ADXL345_REG_WINDOW        = 0x23

ADXL345_REG_ACT_INACT_CTL = 0x27
ADXL345_REG_TAP_AXES      = 0x2A

ADXL345_REG_INT_ENABLE    = 0x2E
ADXL345_REG_INT_MAP       = 0x2F
ADXL345_REG_INT_SOURCE    = 0x30

ADXL345_REG_THRESH_ACT  = 0x24

ADXL345_REG_THRESH_INACT  = 0x25
ADXL345_REG_TIME_INACT    = 0x26

ADXL345_REG_THRESH_FF     = 0x28
ADXL345_REG_TIME_FF       = 0x29

ADXL345_DATARATE_0_10_HZ  = 0x00
ADXL345_DATARATE_0_20_HZ  = 0x01
ADXL345_DATARATE_0_39_HZ  = 0x02
ADXL345_DATARATE_0_78_HZ  = 0x03
ADXL345_DATARATE_1_56_HZ  = 0x04
ADXL345_DATARATE_3_13_HZ  = 0x05
ADXL345_DATARATE_6_25HZ   = 0x06
ADXL345_DATARATE_12_5_HZ  = 0x07
ADXL345_DATARATE_25_HZ    = 0x08
ADXL345_DATARATE_50_HZ    = 0x09
ADXL345_DATARATE_100_HZ   = 0x0A # (default)
ADXL345_DATARATE_200_HZ   = 0x0B
ADXL345_DATARATE_400_HZ   = 0x0C
ADXL345_DATARATE_800_HZ   = 0x0D
ADXL345_DATARATE_1600_HZ  = 0x0E
ADXL345_DATARATE_3200_HZ  = 0x0F
ADXL345_RANGE_2_G         = 0x00 # +/-  2g (default)
ADXL345_RANGE_4_G         = 0x01 # +/-  4g
ADXL345_RANGE_8_G         = 0x02 # +/-  8g
ADXL345_RANGE_16_G        = 0x03 # +/- 16g


ADXL345_DATA_READY         = 0x07,
ADXL345_SINGLE_TAP         = 0x06,
ADXL345_DOUBLE_TAP         = 0x05,
ADXL345_ACTIVITY           = 0x04,
ADXL345_INACTIVITY         = 0x03,
ADXL345_FREE_FALL          = 0x02,
ADXL345_WATERMARK          = 0x01,
ADXL345_OVERRUN            = 0x00


GRAVITY_FACTOR             = 9.80665

class Activitie(object):
  def __init__(self, isOverrun, isWatermark, isFreeFall, isInactivity, isActivity, isDoubleTap, isTap, isDataReady, isActivityOnX, isActivityOnY, isActivityOnZ, isTapOnX, isTapOnY, isTapOnZ):
    self.isOverrun = isOverrun
    self.isWatermark = isWatermark
    self.isFreeFall = isFreeFall
    self.isInactivity = isInactivity
    self.isActivity = isActivity
    self.isDoubleTap = isDoubleTap
    self.isTap = isTap
    self.isDataReady = isDataReady
    self.isActivityOnX = isActivityOnX
    self.isActivityOnY = isActivityOnY
    self.isActivityOnZ = isActivityOnZ
    self.isTapOnX = isTapOnX
    self.isTapOnY = isTapOnY
    self.isTapOnZ = isTapOnZ


class ADXL345(object):
  """ADXL345 triple-axis accelerometer."""

  def __init__(self, address=ADXL345_ADDRESS, i2c=None, **kwargs):
    """Initialize the ADXL345 accelerometer using its I2C interface.
    """
    # Setup I2C interface for the device.
    if i2c is None:
        import Adafruit_GPIO.I2C as I2C
        i2c = I2C
    self._device = i2c.get_i2c_device(address, **kwargs)
    # Check that the acclerometer is connected, then enable it.
    if self._device.readU8(ADXL345_REG_DEVID) == 0xE5:
        self._device.write8(ADXL345_REG_POWER_CTL, 0x08)
    else:
        raise RuntimeError('Failed to find the expected device ID register value, check your wiring.')

    self.clearSettings()

  def clearSettings(self):
    self.set_range(ADXL345_RANGE_2_G)
    self.set_data_rate(ADXL345_DATARATE_100_HZ)

    self._device.write8(ADXL345_REG_THRESH_TAP, 0x00);
    self._device.write8(ADXL345_REG_DUR, 0x00);
    self._device.write8(ADXL345_REG_LATENT, 0x00);
    self._device.write8(ADXL345_REG_WINDOW, 0x00);
    self._device.write8(ADXL345_REG_THRESH_ACT, 0x00);
    self._device.write8(ADXL345_REG_THRESH_INACT, 0x00);
    self._device.write8(ADXL345_REG_TIME_INACT, 0x00);
    self._device.write8(ADXL345_REG_THRESH_FF, 0x00);
    self._device.write8(ADXL345_REG_TIME_FF, 0x00);

    value = self._device.readU8(ADXL345_REG_ACT_INACT_CTL);
    value &= 0b10001000;
    self._device.write8(ADXL345_REG_ACT_INACT_CTL, value);

    value = self._device.readU8(ADXL345_REG_TAP_AXES);
    value &= 0b11111000;
    self._device.write8(ADXL345_REG_TAP_AXES, value);

  def useInterrupt(self, interrupt):

    if (interrupt == 0):
      self._device.write8(ADXL345_REG_INT_MAP, 0x00);
    else:
      self._device.write8(ADXL345_REG_INT_MAP, 0xFF);

    self._device.write8(ADXL345_REG_INT_ENABLE, 0xFF);

  def setActivityXYZ(self, state):
    value = self._device.readU8(ADXL345_REG_ACT_INACT_CTL);

    if (state):
      value |= 0b00111000;
    else:
      value &= 0b11000111;

    self._device.write8(ADXL345_REG_ACT_INACT_CTL, value);

  def setInactivityXYZ(self, state):
    value = self._device.readU8(ADXL345_REG_ACT_INACT_CTL);

    if (state):
      value |= 0b00000111;
    else:
      value &= 0b11111000;

    self._device.write8(ADXL345_REG_ACT_INACT_CTL, value);

  def set_range(self, value):
    """Set the range of the accelerometer to the provided value.  Range value
    should be one of these constants:
      - ADXL345_RANGE_2_G   = +/-2G
      - ADXL345_RANGE_4_G   = +/-4G
      - ADXL345_RANGE_8_G   = +/-8G
      - ADXL345_RANGE_16_G  = +/-16G
    """
    # Read the data format register to preserve bits.  Update the data
    # rate, make sure that the FULL-RES bit is enabled for range scaling
    format_reg = self._device.readU8(ADXL345_REG_DATA_FORMAT) & ~0x0F
    format_reg |= value
    format_reg |= 0x08  # FULL-RES bit enabled
    # Write the updated format register.
    self._device.write8(ADXL345_REG_DATA_FORMAT, format_reg)

  def get_range(self):
    """Retrieve the current range of the accelerometer.  See set_range for
    the possible range constant values that will be returned.
    """
    return self._device.readU8(ADXL345_REG_DATA_FORMAT) & 0x03

  def set_data_rate(self, rate):
    """Set the data rate of the aceelerometer.  Rate should be one of the
    following constants:
      - ADXL345_DATARATE_0_10_HZ = 0.1 hz
      - ADXL345_DATARATE_0_20_HZ = 0.2 hz
      - ADXL345_DATARATE_0_39_HZ = 0.39 hz
      - ADXL345_DATARATE_0_78_HZ = 0.78 hz
      - ADXL345_DATARATE_1_56_HZ = 1.56 hz
      - ADXL345_DATARATE_3_13_HZ = 3.13 hz
      - ADXL345_DATARATE_6_25HZ  = 6.25 hz
      - ADXL345_DATARATE_12_5_HZ = 12.5 hz
      - ADXL345_DATARATE_25_HZ   = 25 hz
      - ADXL345_DATARATE_50_HZ   = 50 hz
      - ADXL345_DATARATE_100_HZ  = 100 hz
      - ADXL345_DATARATE_200_HZ  = 200 hz
      - ADXL345_DATARATE_400_HZ  = 400 hz
      - ADXL345_DATARATE_800_HZ  = 800 hz
      - ADXL345_DATARATE_1600_HZ = 1600 hz
      - ADXL345_DATARATE_3200_HZ = 3200 hz
    """
    # Note: The LOW_POWER bits are currently ignored,
    # we always keep the device in 'normal' mode
    self._device.write8(ADXL345_REG_BW_RATE, rate & 0x0F)

  def get_data_rate(self):
    """Retrieve the current data rate.  See set_data_rate for the possible
    data rate constant values that will be returned.
    """
    return self._device.readU8(ADXL345_REG_BW_RATE) & 0x0F

  def set_int_enable(self, value):
    self._device.write8(ADXL345_REG_INT_ENABLE, value)

  def get_int_enable(self):
    return self._device.readU8(ADXL345_REG_INT_ENABLE)

  def set_int_map(self, value):
    self._device.write8(ADXL345_REG_INT_MAP, value)

  def get_int_map(self):
    return self._device.readU8(ADXL345_REG_INT_MAP)

  def set_thresh_act(self, value):
    self._device.write8(ADXL345_REG_THRESH_ACT, value)

  def get_thresh_act(self):
    return self._device.readU8(ADXL345_REG_THRESH_ACT)

  def set_thresh_inact(self, value):
    self._device.write8(ADXL345_REG_THRESH_INACT, value)

  def get_thresh_inact(self):
    return self._device.readU8(ADXL345_REG_THRESH_INACT)

  def set_time_inact(self, value):
    self._device.write8(ADXL345_REG_TIME_INACT, value)

  def get_time_inact(self):
    return self._device.readU8(ADXL345_REG_TIME_INACT)  

  def set_thresh_ff(self, value):
    self._device.write8(ADXL345_REG_THRESH_FF, value)

  def get_thresh_ff(self):
    return self._device.readU8(ADXL345_REG_THRESH_FF)

  def set_time_ff(self, value):
    self._device.write8(ADXL345_REG_TIME_FF, value)

  def get_time_ff(self):
    return self._device.readU8(ADXL345_REG_TIME_FF)

  def read(self):
    """Read the current value of the accelerometer and return it as a tuple
    of signed 16-bit X, Y, Z axis values.
    """
    raw = self._device.readList(ADXL345_REG_DATAX0, 6)
    return struct.unpack('<hhh', raw)

  def readNormalize(self):
    xAxis = self._device.readU16(ADXL345_REG_DATAX0)
    yAxis = self._device.readU16(ADXL345_REG_DATAY0)
    zAxis = self._device.readU16(ADXL345_REG_DATAZ0)

    nXAxis = xAxis * 0.004 * GRAVITY_FACTOR
    nYAxis = yAxis * 0.004 * GRAVITY_FACTOR
    nZAxis = zAxis * 0.004 * GRAVITY_FACTOR

    return nXAxis, nYAxis, nZAxis

  def readActivites(self):
    data = self._device.readU8(ADXL345_REG_INT_SOURCE)

    isOverrun = ((data >> ADXL345_OVERRUN) & 1);
    isWatermark = ((data >> ADXL345_WATERMARK) & 1);
    isFreeFall = ((data >> ADXL345_FREE_FALL) & 1);
    isInactivity = ((data >> ADXL345_INACTIVITY) & 1);
    isActivity = ((data >> ADXL345_ACTIVITY) & 1);
    isDoubleTap = ((data >> ADXL345_DOUBLE_TAP) & 1);
    isTap = ((data >> ADXL345_SINGLE_TAP) & 1);
    isDataReady = ((data >> ADXL345_DATA_READY) & 1);

    data = self._device.readU8(ADXL345_REG_ACT_TAP_STATUS);

    isActivityOnX = ((data >> 6) & 1);
    isActivityOnY = ((data >> 5) & 1);
    isActivityOnZ = ((data >> 4) & 1);
    isTapOnX = ((data >> 2) & 1);
    isTapOnY = ((data >> 1) & 1);
    isTapOnZ = ((data >> 0) & 1);

    return Activities(isOverrun, isWatermark, isFreeFall, isInactivity, isActivity, isDoubleTap, isTap, isDataReady, isActivityOnX, isActivityOnY, isActivityOnZ, isTapOnX, isTapOnY, isTapOnZ)

GPIO.setmode(GPIO.BOARD)

GPIO_INT1 = 8
GPIO_INT2 = 10

GPIO.setup(GPIO_INT1, GPIO.OUT)
GPIO.setup(GPIO_INT2, GPIO.OUT)

accel = ADXL345()

accel.set_thresh_act(0x20)
accel.set_thresh_inact(0x20)
accel.set_time_inact(0x05)

accel.setActivityXYZ(True); 
accel.setInactivityXYZ(True); 

accel.useInterrupt(0);

#accel.set_int_enable(0x14)
#accel.set_int_map(0x10)
#accel.set_thresh_act(0x21)
#accel.set_thresh_ff(0x06)
#accel.set_time_ff(0x14)

fall = None
activity = None

ACTIVITY_DETECTION_INTERVAL = 150
timer = 0

try:
  while True:
    x, y, z = accel.readNormalize()
    print('X={0}, Y={1}, Z={2}'.format(x, y, z))

    activities = accel.readActivites()

    if (activities.isActivity):
      print("Activity Detected");
    if (activities.isInactivity):
      print("Inactivity Detected");

    #fall = GPIO.input(GPIO_INT1)
    #activity = GPIO.input(GPIO_INT2)

    #if(fall):
    #  print("Fall Detected!")
    #if(activity and timer > ACTIVITY_DETECTION_INTERVAL):
    #  print("Activity Detected!")
    #  timer = 0

    time.sleep(0.2)
    #timer += 1
except KeyboardInterrupt:
  pass
finally:
  GPIO.cleanup()