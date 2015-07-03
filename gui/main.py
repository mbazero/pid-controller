import parser
import config
import device_manager
import model
import view
import controller

'''
Parse hdl parameters
'''
parser = parser.HDLParser()
parser.parse(config.params_path)
parser.parse(config.ep_map_path)
params = parser.get_params()

'''
Instantiate FPGA device manager
'''
fpga = device_manager.DeviceManager(config.pid_bit_file, config.serial,
        config.sys_clk_freq, config.adc_clk_freq, params)

#if fpga.init_device() == False:
    #exit

'''
Instantiate MVC and start GUI
'''
model = model.Model(params)
view = view.View(params)
controller = controller.Controller(view, model, fpga, params)
view.run()
