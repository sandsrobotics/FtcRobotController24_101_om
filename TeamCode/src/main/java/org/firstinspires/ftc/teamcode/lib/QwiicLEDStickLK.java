package org.firstinspires.ftc.teamcode.lib;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import androidx.annotation.ColorInt;

@I2cDeviceType()
@DeviceProperties(name = "QWIIC LED Stick LK", description = "Sparkfun QWIIC LED Stick LK", xmlTag = "QWIIC_LED_STICK")
public class QwiicLEDStick extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> implements OpModeManagerNotifier.Notifications {

    private enum Commands {
        CHANGE_LED_LENGTH(0x70),
        WRITE_SINGLE_LED_COLOR(0x71),
        WRITE_ALL_LED_COLOR(0x72),
        WRITE_RED_ARRAY(0x73),
        WRITE_GREEN_ARRAY(0x74),
        WRITE_BLUE_ARRAY(0x75),
        WRITE_SINGLE_LED_BRIGHTNESS(0x76),
        WRITE_ALL_LED_BRIGHTNESS(0x77),
        WRITE_ALL_LED_OFF(0x78),
        CHANGE_DEFAULT_BRIGHTNESS (0x80),
        //CHANGE_LED_TYPE (0x81),
        CHANGE_MIRROR_MODE (0x82),
        WRITE_COLOR_LENGTH (0x90),
        WRITE_COLOR_LENGTH_WITH_BLANK (0x91),
        WRITE_COLOR_LENGTH2 (0x92),
        WRITE_COLOR_LENGTH2_WITH_BLANK (0x93),
        CONTROL_ANIMATION (0xA0),
        FORCE_WDT_REBOOT (0xB0);
        int bVal;

        Commands(int bVal) {
            this.bVal = bVal;
        }
    }

    /** LK Extended Functions */
    public void setColorGroup(int startLED, int numLED, @ColorInt int color) {
        setColorGroup(startLED, numLED, color, false);
    }
    public void setColorGroup(int startLED, int numLED, @ColorInt int color, boolean blankFirst ) {
        byte[] data = new byte[5];
        data[0] = (byte) startLED;
        data[1] = (byte) numLED;
        data[2] = (byte) Color.red(color);
        data[3] = (byte) Color.green(color);
        data[4] = (byte) Color.blue(color);
        if (!blankFirst) writeI2C(Commands.WRITE_COLOR_LENGTH, data);
        else writeI2C(Commands.WRITE_COLOR_LENGTH_WITH_BLANK, data);
    }
    public void setColorGroupX2(int startLED, int numLED, @ColorInt int color, int startLED2, int numLED2, @ColorInt int color2 ) {
        setColorGroupX2(startLED, numLED, color, startLED2, numLED2, color2,false);
    }
    public void setColorGroupX2(int startLED, int numLED, @ColorInt int color, int startLED2, int numLED2, @ColorInt int color2, boolean blankFirst ) {
        byte[] data = new byte[10];
        data[0] = (byte) startLED;
        data[1] = (byte) numLED;
        data[2] = (byte) Color.red(color);
        data[3] = (byte) Color.green(color);
        data[4] = (byte) Color.blue(color);
        data[5] = (byte) startLED2;
        data[6] = (byte) numLED2;
        data[7] = (byte) Color.red(color2);
        data[8] = (byte) Color.green(color2);
        data[9] = (byte) Color.blue(color2);
        if (!blankFirst) writeI2C(Commands.WRITE_COLOR_LENGTH2, data);
        else writeI2C(Commands.WRITE_COLOR_LENGTH2_WITH_BLANK, data);
    }
    public void setDefaultBrightness(int brightness) {  // this writes to EEPROM
        if (brightness < 1 || brightness > 31) return;
        byte[] data = new byte[1];
        data[0] = (byte) brightness;
        writeI2C(Commands.CHANGE_DEFAULT_BRIGHTNESS, data);
    }
    public void setMirrorMode(int mode) {  // this writes to EEPROM
        if (mode < 0 || mode > 2) return;
        byte[] data = new byte[1];
        data[0] = (byte) mode;
        writeI2C(Commands.CHANGE_MIRROR_MODE, data);
    }
    public void setAnimationOnOff(int mode) {
        if (mode < 0 || mode > 1) return;
        byte[] data = new byte[1];
        data[0] = (byte) mode;
        writeI2C(Commands.CONTROL_ANIMATION, data);
    }
    public void forceReboot() {
        byte[] data = new byte[1];
        data[0] = (byte) 0;
        writeI2C(Commands.FORCE_WDT_REBOOT, data);
    }

    /**
     * Change the color of a specific LED
     *
     * @param position which LED to change (1 - 255)
     * @param color    what color to set it to
     */
    public void setColor(int position, @ColorInt int color) {
        byte[] data = new byte[4];
        data[0] = (byte) position;
        data[1] = (byte) Color.red(color);
        data[2] = (byte) Color.green(color);
        data[3] = (byte) Color.blue(color);
        writeI2C(Commands.WRITE_SINGLE_LED_COLOR, data);
    }

    /**
     * Change the color of all LEDs to a single color
     *
     * @param color what the color should be
     */
    public void setColor(@ColorInt int color) {
        byte[] data = new byte[3];
        data[0] = (byte) Color.red(color);
        data[1] = (byte) Color.green(color);
        data[2] = (byte) Color.blue(color);
        writeI2C(Commands.WRITE_ALL_LED_COLOR, data);
    }

    /**
     * Send a segment of the LED array
     *
     * @param cmd    command to send
     * @param array  the values (limited from 0..255)
     * @param offset the starting value (LED only, array starts at 0)
     * @param length the length to send
     */
    private void sendSegment(Commands cmd, int[] array, int offset, int length) {
        byte[] data = new byte[length + 2];
        data[0] = (byte) length;
        data[1] = (byte) offset;

        for (int i = 0; i < length; i++) {
            data[2 + i] = (byte) array[i];
        }
        writeI2C(cmd, data);
    }

    /**
     * Change the color of an LED color segment
     *
     * @param colors what the colors should be
     * @param offset where in the array to start
     * @param length length to send (limited to 12)
     */
    private void setLEDColorSegment(@ColorInt int[] colors, int offset, int length) {
        int[] redArray = new int[length];
        int[] greenArray = new int[length];
        int[] blueArray = new int[length];

        for (int i = 0; i < colors.length; i++) {
            redArray[i] = Color.red(colors[i + offset]);
            greenArray[i] = Color.green(colors[i + offset]);
            blueArray[i] = Color.blue(colors[i + offset]);
        }
        sendSegment(Commands.WRITE_RED_ARRAY, redArray, offset, length);
        //sleep(3);
        sendSegment(Commands.WRITE_GREEN_ARRAY, greenArray, offset, length);
        //sleep(3);
        sendSegment(Commands.WRITE_BLUE_ARRAY, blueArray, offset, length);
    }

    /**
     * Change the color of all LEDs using arrays
     *
     * @param colors array of colors to set lights to
     */
    public void setColors(@ColorInt int[] colors) {
        int length = colors.length;

        int numInLastSegment = length % 12;
        int numSegments = length / 12;
        for (int i = 0; i < numSegments; i++) {
            setLEDColorSegment(colors, i * 12, 12);
        }
        setLEDColorSegment(colors, numSegments * 12, numInLastSegment);
    }

    /**
     * Set the brightness of an individual LED
     *
     * @param number     which LED to change (1-255)
     * @param brightness brightness level (0-31)
     */
    public void setBrightness(int number, int brightness) {
        byte[] data = new byte[2];
        data[0] = (byte) number;
        data[1] = (byte) brightness;
        writeI2C(Commands.WRITE_SINGLE_LED_BRIGHTNESS, data);
    }

    /**
     * Set the brightness of all LEDs
     *
     * @param brightness brightness level (0-31)
     */
    public void setBrightness(int brightness) {
        byte[] data = new byte[1];
        data[0] = (byte) brightness;
        writeI2C(Commands.WRITE_ALL_LED_BRIGHTNESS, data);
    }

    /**
     * Turn all LEDS off...
     */
    public void turnAllOff() {
        setColor(0);
    }

    /**
     * Change the length of the LED strip
     *
     * @param newLength 1 to 100 (longer than 100 not supported)
     */
    public void changeLength(int newLength) {
        byte[] data = new byte[1];
        data[0] = (byte) newLength;
        writeI2C(Commands.CHANGE_LED_LENGTH, data);
    }

    private void writeI2C(Commands cmd, byte[] data) {
        deviceClient.write(cmd.bVal, data, I2cWaitControl.WRITTEN);
    }


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Qwiic LED Strip";
    }

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x23);

    public QwiicLEDStick(I2cDeviceSynchSimple deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {
        turnAllOff();
    }

    @Override
    public void onOpModePreStart(OpMode opMode) {
        turnAllOff();
    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        turnAllOff();
    }

}