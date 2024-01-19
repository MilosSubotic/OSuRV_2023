
if not 'SIM' in globals():
	SIM = False
if not 'ARM' in globals():
	ARM = 's3a'

import subprocess
def run_for_stdout(str_cmd):
	r = subprocess.run(str_cmd.split(), stdout = subprocess.PIPE)
	return r.stdout.decode('utf-8')
on_rpi = run_for_stdout('lsb_release -i') == 'Distributor ID:	Raspbian\n'

arm_main = f'{ARM}_main'
if not on_rpi:
	if ARM == 's3a':
		arm_main = f'{ARM}_serial_main'

_settings = {
	'SIM': SIM,
	'ARM': ARM,
	'arm_main': arm_main,
}



_layout = [
	{
		'drv':
		# Tuple is vert pane split, while list is horiz.
		(
			['driver', 'app'],
			'dmesg',
		)
	},
]

# Key is target name, value is list of batches.
_targets = {
	# Done before everything.
	'setup': [
		{
			'driver': 'cd ../../SW/Driver/motor_ctrl/',
			'app': 'cd ../../SW/App/wc_rosless_teleop/test/',
			'dmesg': 'cd ../../SW/Driver/motor_ctrl/',
		},
		
	],
	'build' : [
		{
			'driver' : '''
				make
				''',
			'app' : '''
				./waf configure
				./waf build
				''',
		},
	],
	'run': [
		{
			'driver' : '''
				make stop
				make start
				''',
			'dmesg' : '''
				tmux_no_wait
				dmesg -w
				''',
			'app' : '''
				./build/test_bldc 0
				./build/test_chassis NO_ENTER
				''',
		},
	],

	

}

# Do not wait in last batch.
last_batch = _targets['run'][-1]
no_wait_last_batch = {}
for pane, cmds in last_batch.items():
	cmds = 'tmux_no_wait\n' + cmds
	no_wait_last_batch[pane] = cmds
_targets['run'][-1] = no_wait_last_batch

_dependencies = {
	'build': ['setup'],
	'run': ['setup'],
}


