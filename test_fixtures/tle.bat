REM PID Controller Batch File
REM $Rev$ $Date$

REM Edit path for settings32/64, depending on architecture
call %XILINX%\..\settings64.bat

fuse -intstyle ise ^
     -incremental ^
     -lib unisims_ver ^
     -lib unimacro_ver ^
     -lib xilinxcorelib_ver ^
     -i ../ok_sim ^
     -i ../../pidc ^
     -o tle.exe ^
     -prj tle.prj ^
     work.pid_controller_tf work.glbl
tle.exe -gui -tclbatch tle.tcl -wdb tle.wdb
