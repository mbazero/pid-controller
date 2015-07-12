import hdl_parser
import config
import device_manager
import model
import view
import controller

'''
Parse hdl parameters
'''
hdlp = hdl_parser.HDLParser()
params = hdlp.get_params()

for header in config.header_list:
    hdlp.parse(header)

'''
Instantiate FPGA device manager
'''
fpga = device_manager.DeviceManager(config.pid_bit_file, config.serial,
        config.sys_clk_freq, config.adc_clk_freq, params)

if fpga.init_device() == False:
    exit

'''
Instantiate MVC and start GUI
'''
model = model.Model(params)
view = view.View(params)
controller = controller.Controller(view, model, fpga, params, config.pconfig)
view.run()
