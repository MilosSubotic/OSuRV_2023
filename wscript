#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
@author: Milos Subotic <milos.subotic.sm@gmail.com>
@license: MIT

@brief: Waf script just for distclean and dist commands.
'''

###############################################################################

import os
import sys
sys.path.append('./Common/Scripts')
import common_waf

###############################################################################

APPNAME = os.path.basename(os.getcwd())

top = '.'

def distclean(ctx):
	common_waf.distclean(ctx)
	
def dist(ctx):
	common_waf.dist(ctx)

###############################################################################
