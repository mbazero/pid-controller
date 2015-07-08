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
  * Prefer
2. On the `Project Type` window select `RTL Project`
3. On the `Add Sources` screen:
  * add all `.v` and `.vh` files in directory `pidc/`
  * add all `.v` and `.ngc` files in directory `pidc/ok_library`
4. On the `Add Existing IP` screen add all `.xco` files in directory `pidc/ip_core`
5. On the `Add Constraints` screen add file `xem6010.ucf`
6. On the `Default Part screen select part `XC6SLX45FGG484-2`

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
