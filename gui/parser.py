'''
Class to parse HDL parameters into python parameters
'''
class HDLParser:
    def __init__(self):
        self.params = ParamContainer()

    def parse(self, map_path):
        chars_to_remove = [' ', '\t', '\n']
        infile = open(map_path)

        for line in infile:
            # skip line if it does not define a parameter
            if line.find('parameter') != 0 and line.find('localparam') != 0:
                continue

            # remove extraneous substrings and characters from line
            line = line.replace('parameter', '')
            line = line.replace('localparam', '')
            line = line.replace('8\'', '\'')
            line = line.replace('16\'', '\'')
            line = line.replace('32\'', '\'')
            line = line[0:line.find(';')] # strip everything beyond the semicolon
            line = line.translate(None, ''.join(chars_to_remove)) # remove spaces and tabs

            # split line into name and value strings
            line = line.lower()
            [name, value_str] = line.split('=')

            # convert value radix identifiers from verilog to python format
            value_str = value_str.lower()
            value_str = value_str.replace('\'h', '0x')
            value_str = value_str.replace('\'d', '')
            value_str = value_str.replace('\'b', '0b')

            # generate python parameter
            value = eval(value_str, self.params.__dict__)
            self.params.set_attribute(name, value)

    def get_params(self):
        return self.params


'''
Container class to hold parsed parameters
'''
class ParamContainer:
    def set_attribute(self, name, value):
        setattr(self, name, value)

    def get_attribute(self, name):
        return getattr(name)
