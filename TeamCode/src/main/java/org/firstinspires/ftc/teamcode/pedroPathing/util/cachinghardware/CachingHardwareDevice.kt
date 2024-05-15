package org.firstinspires.ftc.teamcode.pedroPathing.util.cachinghardware

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer

abstract class CachingHardwareDevice protected constructor(val hardwareDevice: HardwareDevice) : HardwareDevice {

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the device's manufacturer
     */
    override fun getManufacturer(): Manufacturer {
        return hardwareDevice.manufacturer
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    override fun getDeviceName(): String {
        return hardwareDevice.deviceName
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    override fun getConnectionInfo(): String {
        return hardwareDevice.connectionInfo
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    override fun getVersion(): Int {
        return hardwareDevice.version
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    override fun resetDeviceConfigurationForOpMode() {
        hardwareDevice.resetDeviceConfigurationForOpMode()
    }

    /**
     * Closes this device
     */
    override fun close() {
        hardwareDevice.close()
    }
}