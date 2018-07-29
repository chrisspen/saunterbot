#!/usr/bin/env python
"""
An attempt to introduce a third "do nothing" action.

Not much beneficial effect. I had to make the range for this action very small,
otherwise the search never converged. And even then, the results aren't completely stable
and decay after a few minutes.
"""

from carttableangular import CartTableAngularEnv
from search import multi_hill_search, demo

env = CartTableAngularEnv()

seed = [0.30625239348824673, 0.312614687196611, 0.9243082375560138, 1.425081839579726]
env.set_actions(3)
env.cog_offset = 0 # centered weight, solved by MHS
# env.cog_offset = env.length2/2. # offset weight
# env.cog_offset = env.length2 # large offset weight, unsolvable
score, params, episodes = multi_hill_search(env=env, seed=seed, reward_threshold=3000)
try:
    demo(env, params)
except KeyboardInterrupt:
    print()
print('bestscore:', score)
print('bestparams:', list(params))
print('episodes:', episodes)
