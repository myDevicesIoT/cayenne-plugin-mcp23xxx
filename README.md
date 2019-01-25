# Cayenne MCP23XXX Plugin
A plugin allowing the [Cayenne Pi Agent](https://github.com/myDevicesIoT/Cayenne-Agent) to read data from MCP23XXX devices (PiFace Digital, MCP23008, MCP23009, MCP23017, MCP23018, MCP23S08, MCP23S09, MCP23S17, MCP23S18) and display it in the [Cayenne Dashboard](https://cayenne.mydevices.com).

## Requirements
### Hardware
* [Rasberry Pi](https://www.raspberrypi.org).
* An MCP23XXX extension, e.g. [MCP23017](https://www.adafruit.com/product/732), or a [PiFace Digital](http://www.piface.org.uk/products/piface_digital/).

### Software
* [Cayenne Pi Agent](https://github.com/myDevicesIoT/Cayenne-Agent). This can be installed from the [Cayenne Dashboard](https://cayenne.mydevices.com).
* [Git](https://git-scm.com/).

## Getting Started
### 1. Installation

   From the command line run the following commands to install this plugin.
   ```
   cd /etc/myDevices/plugins
   sudo git clone https://github.com/myDevicesIoT/cayenne-plugin-mcp23xxx.git
   ```

### 2. Modifying the plugin

   Specify the device you are using by setting the `class` value under the `MCP` section in the `cayenne_mcp23xxx.plugin` file.
   By default this is set to `MCP23017` but it can be set to use any of the classes in the `cayenne_mcp23xxx` module.
   
   If your device has fewer channels than the `MCP23017` or you do not want the raw channel values to be displayed in the Cayenne 
   dashboard you can disable any of the individual input sections in `cayenne_mcp23xxx.plugin`.

   By default the plugin alternates between input and output channels. To specify different functions for specific channels you
   can modify the `class` for that section to specify `DigitalInput` or `DigitalOutput`.

### 3. Restarting the agent

   Restart the agent so it can load the plugin.
   ```
   sudo service myDevices restart
   ```
   Temporary widgets for the plugin should now show up in the [Cayenne Dashboard](https://cayenne.mydevices.com). You can make them permanent by clicking the plus sign.

   NOTE: If the temporary widgets do not show up try refreshing the [Cayenne Dashboard](https://cayenne.mydevices.com) or restarting the agent again using `sudo service myDevices restart`.