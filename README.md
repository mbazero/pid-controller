PID Controller
===============

Multichannel digital PID controller for use with Opal Kelly XEM6010 FPGA
integration module and Duke MIST breakout board. Controller can be
configured to drive DAC (Texas Instruments DAC8568) or DDS (Analog
Devices AD9912) outputs. A Python GUI is provided for real time
controller configuration and monitoring.

### Usage

## Quick launch
1. `git clone https://github.com/madams2419/pid-controller`
2. `cd /pid-controller/gui`
3. `python alpha.py`
4. Set channel parameters in GUI as desired
5. Press `Set DAC Reference`
6. Press `Start ADC`
7. Press `Activate Channel` for each active channel

## Creating an ISE project to regenerate bit file
1. Open Xilinx ISE and select `File -> New Project`
2. In `Project Settings` specify the following options:
  * Family  = `Spartan 6`
  * Device  = `XC6SLX45`
  * Package = `FGG484`
  * Speed   = `-2`
3. Select `Project -> Add Source` and:
  * Add all files in directory `pidc/`
  * Add all files in directory `pidc/ok_library`
  * Add files `fifo_16.xco`, `fifo_19.xco` and `fifo_21.xco` from directory `pidc/ip_core`
4. In the `Heirarchy` pane, right click `pid_controller.v` and click
   `Set as Top Module`

## Editing Existing Xilinx Generated Cores
1. Open Xilinx ISE and select `Tools -> Core Generator`
2. When the Core Generator window opens select `File -> Open Project`
   and select file `coregen.cgp` from directory `pidc/ip_core`
3. Right click the name of the core you want to edit in the `Project IP`
   pane and click `Recustomize and Generate (Under Original Project
   Settings)`

### Notes

* FGPA bit file is located at `/gui/pid_controller.bit`. FPGA is
  programmed with bit file with each run of the GUI.
* If there are errors connected to the Opal Kelly module, they will be
  displayed in console on GUI launch.
* If Verilog changes are made, a new bit file must be generated. To do
  this, import Verilog source files into a Xilinx ISE Design Suite
  project and select "Generate bit file." When the process completes,
  rename the bit file to `pid_controller.bit` and overwrite the old
  file.
