import hdl_parser
import config
import io_config
import device_manager
import model
import view
import controller

'''
Parse HDL parameters
'''
hdl_parser = hdl_parser.HDLParser()
params = hdl_parser.get_params()

for header in config.header_list:
    hdl_parser.parse(header)

'''
Get I/O configuration
'''
io_config = io_config.IOConfig(params)

'''
Instantiate FPGA device manager
'''
fpga = device_manager.DeviceManager(config.pid_bit_file, config.serial,
        config.adc_clk_freq, config.sys_clk_freq, params)

if fpga.init_device() == False:
    exit

'''
Instantiate MVC and start GUI
'''
model = model.Model(io_config, params)
view = view.View(io_config, params)
controller = controller.Controller(view, model, fpga, io_config, params,
        config.config_path)
view.run()
