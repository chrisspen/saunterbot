#!/usr/bin/env python

from carttableangular import CartTableAngularEnv
from search_carttableangular_pid import PID
from search import demo

env = CartTableAngularEnv()
env.cog_offset = 0
params = [19.883364107899475, -8.554889746271598, 6.14581940379808]
demo(env, params, eval_cls=PID)
