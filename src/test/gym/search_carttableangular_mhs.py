#!/usr/bin/env python
"""
Based on http://kvfrans.com/simple-algoritms-for-solving-cartpole/
"""

from carttableangular import CartTableAngularEnv
from search import multi_hill_search, demo

env = CartTableAngularEnv()

seed = None
env.cog_offset = 0 # centered weight, solved by MHS
# env.cog_offset = env.length2/2. # offset weight
# env.cog_offset = env.length2 # large offset weight, unsolvable
score, params, episodes = multi_hill_search(env=env, seed=seed, reward_threshold=3000) # good, usually finds a good solution under 1k episodes
try:
    demo(env, params)
except KeyboardInterrupt:
    print()
print('bestscore:', score)
print('bestparams:', list(params))
print('episodes:', episodes)

# bestscore: 2000.0 # from MHS, stays perfectly upright
# bestparams: [0.16265936211407533, 0.024178259995452242, 0.6212972106657324, 0.5684683195540158]
# env.cog_offset = 0 # centered weight
# demo([0.16265936211407533, 0.024178259995452242, 0.6212972106657324, 0.5684683195540158])

# bestscore: 2000.0 # from MHS, stays perfectly upright
# bestparams: [0.14903475091456297, 0.03322683626232743, 0.5232701018833608, 0.41386225435667257]
# episodes: 23
# env.cog_offset = 0 # centered weight
# demo([0.14903475091456297, 0.03322683626232743, 0.5232701018833608, 0.41386225435667257])

# bestscore: 3992.9439618453825 # MHS, good
# episodes: 161
# env.cog_offset = 0 # centered weight
# demo([0.23521520244828986, 0.047318280877045815, 0.9386940808666806, 0.5162175266755475])

# bestscore: 3287.0307115969667 # MHS, good and stable but heavily bent
# episodes: 28
# env.cog_offset = env.length2/2. # offset weight
# demo([0.1896184674558783, 0.09039795516005612, 0.8948622934994588, 0.6044337203784478])

# bestscore: 3435.907323413672, # MHS, good and stable but bent
# episodes: 999
# env.cog_offset = env.length2/2. # offset weight
# demo([0.17983774835249908, 0.042358647889821285, 1.032064033964714, 0.3455176789838385])

# env.cog_offset = env.length2/2. # offset weight
# demo([0.6458582497800833, -0.09901549169812451, 0.6287865958102437, -0.6127295330284901])

#good, but need to fix non-parallel angle
# env.cog_offset = env.length2/2. # offset weight
# demo([0.24440381698868016, 0.016651776934314112, 0.976185166444728, 0.2306152531114938])
