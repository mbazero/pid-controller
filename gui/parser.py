class Parse:

    def __init__(self, map_path):
        self._parse(map_path)

    def _parse(self, map_path):
        chars_to_remove = [' ', '\t', '\n']
        infile = open(map_path)
        for line in infile:
            # skip line if it does not define a parameter
            if(line.find('parameter') != 0): continue

            # remove extraneous substrings and characters from line
            line = line.replace('parameter', '')
            line = line.replace('8\'', '')
            line = line.replace('16\'', '')
            line = line.replace('32\'', '')
            line = line[0:line.find(';')]
            line = line.translate(None, ''.join(chars_to_remove))

            # split line at equals
            name_val = line.split('=')

            # parse name
            name = name_val[0].lower()

            # parse value
            if name_val[1].find('h') >= 0 :
                val = int(name_val[1].replace('h', ''), 16)
            elif name_val[1].find('d') >= 0:
                val = int(name_val[1].replace('d', ''), 10)
            elif name_val[1].find('b') >= 0:
                val = int(name_val[1].replace('b', ''), 2)
            else :
                val = int(name_val[1], 10)

            # generate new global variable
            setattr(self, name, val)
