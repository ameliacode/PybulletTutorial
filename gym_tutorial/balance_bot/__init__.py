from gym.envs.registration import register

#register: tells gym where to find the environment class
#argument
## entry_point: specify subclass for env

register(
    id='balancebot-v0',
    entry_point='balance_bot.envs:BalancebotEnv',
)