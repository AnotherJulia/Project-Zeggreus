with open("ONBOARD_BLACKBOX_raw_serial_2.txt") as f:
	lines = f.readlines()
	formatted = "".join([line.split("> ")[1] for line in lines])

	with open("Out.csv", "w") as p:
		p.write(formatted)