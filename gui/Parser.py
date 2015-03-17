class EndpointMap:

	def __init__(self, map_path):
		parse_map(map_path)

	def _parse_map(self, map_path):
		chars_to_remove = [' ', '\t', '\n', ';']
		infile = open(map_path)
		for line in infile:
			# remove extraneous substrings and characters from line
			line = line.replace('parameter', '')
			line = line.replace('8\'h', '')
			line = line.translate(None, ''.join(chars_to_remove))

			# parse endpoint name and value
			name_val = line.split('=')
			name = name_val[0]
			val = int(name_val[1], 16)

			# generate a new variable
			globals()[name] = val




