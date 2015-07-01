TODO
==========
1. Debug wire-out check in simulation
2. Test DDS channels in simulation

Issues
==========
* okc opens first device found. need to have option to choose the device
* two output channels can't read from the same input concurrently (the
gui fucks up)
* I think phase, frequency, and amplitude are also unsigned. If this is
the case you need to modify the data lengths like you did with the DAC
channels.

Questions
==========
* What should happen when a channel is deactivated?
	> Should everything reset?
