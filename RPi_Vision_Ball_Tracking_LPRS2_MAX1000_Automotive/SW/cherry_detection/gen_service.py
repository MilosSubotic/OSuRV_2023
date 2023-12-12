#!/usr/bin/env python
# encoding: utf-8

'''
@author: Milos Subotic <milos.subotic.sm@gmail.com>
@license: MIT

'''

###############################################################################

import os
import sys
import argparse

###############################################################################

def gen_service(
    service_file,
    service_bin
):

	#service_name, _ = os.path.splitext(os.path.basename(service_bin))

	s = '''[Unit]
Description=To run service on startup
After=network.target

[Service]
Type=simple
User=pi
Environment="DISPLAY=:0"
Environment="/home/pi/.Xauthority"
ExecStart={service_bin}

[Install]
WantedBy=multi-user.target
'''.format(
		service_bin = service_bin
	)
	with open(service_file, 'w') as f:
		f.write(s)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument(
		'-o',
		'--output',
		required = True,
		help = 'Output service file'
	)
	parser.add_argument(
		'-i',
		'--input',
		required = True,
		help = 'Installed program'
	)
	args = parser.parse_args()

	gen_service(
		args.output,
		args.input
	)

###############################################################################
