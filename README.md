PID Controller
===============

Multichannel digital PID controller for use with Opal Kelly XEM6010 FPGA
integration module and Duke MIST breakout board. Controller can be
configured to drive DAC (Texas Instruments DAC8568) or DDS (Analog
Devices AD9912) outputs. A Python GUI is provided for real time
controller configuration and monitoring.

## Quick launch
1. Power XEM6010 and attach to computer via USB
1. `git clone https://github.com/madams2419/pid-controller`
2. `cd /pid-controller/gui`
3. Open `config.py`
  * Set path to default configuration file or leave blank to use FPGA
  initial settings.
  * Set path to PID controller bit file.
3. Run `python main.py`

## GUI Software Requirements
The GUI requires Python 2.7, and the PyQt, PyQtGraph, and Numpy
packages. The [Anaconda](http://continuum.io/downloads) Python
distribution includes all of the required packages except PyQtGraph.
PyQtGraph can be downloaded from [here](http://www.pyqtgraph.org/).

## Default Bit File
* Located at `/gui/pid_controller.bit`
* Supports 8 DAC channels and 4 DDS channels. DDS channels 1 and 2 are
  mapped to output connector 1. Channels 3 and 4 are mapped to connector
  2.
* Uses 128-bit registers for PID computations. This is quite large and
  can be reduced substantially to save FPGA area.

## Creating an ISE project
1. Open Xilinx ISE and select `File -> New Project`
2. In `Project Settings` specify the following options:
  * Family  = `Spartan 6`
  * Device  = `XC6SLX45`
  * Package = `FGG484`
  * Speed   = `-2`
3. Select `Project -> Add Source` and:
  * Add all files in directory `pidc/`
  * Add all files in directory `pidc/ok_library`
  * Add files `adc_fifo.xco`, `idp_fifo.xco`, `dac_fifo.xco`, and
    `pipe_fifo.xco` from directory `pidc/ip_core`
4. In the `Heirarchy` pane, right click `pid_controller.v` and click
   `Set as Top Module`

## Editing Existing Xilinx Generated Cores
1. Open Xilinx ISE and select `Tools -> Core Generator`
2. When the Core Generator window opens select `File -> Open Project`
   and select file `coregen.cgp` from directory `pidc/ip_core`
3. Right click the name of the core you want to edit in the `Project IP`
   pane and click `Recustomize and Generate (Under Original Project
   Settings)`

## Changing HDL Parameters
* The Verilog code is fully parametrized. Parameters are stored in
  `parameters.vh`, `init.vh`, and `ep_map.vh`.
  * `parameters.vh` specifies general controller parameters.
  * `init.vh` specifies controller initial state.
  * `ep_map.vh` specifies Opal Kelly endpoints and controller memory
    write addresses.
* Parameters of particular interest are `N_DDS` and `W_COMP` in
  `parameters.vh`.
  * `N_DDS`
    * Specifies the number of DDS controllers to instantiate
    * Can be increased to support more than the default 4 DDS channels
      or decreased to save on FPGA space.
    * If the value is increased, the constraints file (`xem6010.ucf`)
      must be edited to map the new DDS channels to output pins. Look to
      the existing mappings for channels 1-4 to see how this is done.
  * `W_COMP`
    * Specifies the width of computation registers used in the PID
      pipeline.
    * Can be reduced to save FPGA space as the expense of a reduction in
      precision.
* Bit file must be regenerated if any parameters are changed
* The GUI parses all three parameter files at runtime to provide
  consistency between FGPA and GUI state.

### Notes

* If there are errors connected to the Opal Kelly module, they will be
  displayed in console on GUI launch.
* If Verilog changes are made, a new bit file must be generated. To do
  this, import Verilog source files into a Xilinx ISE Design Suite
  project and select "Generate bit file." When the process completes,
  rename the bit file to `pid_controller.bit` and overwrite the old
  file.
