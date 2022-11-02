import gym

env = gym.make('CartPole-v1', render_mode="human")

observation = env.reset()

for t in range(10):

    done = False
    while done == False:
        env.render()
        action = env.action_space.sample()
        observation, reward, done, truncated, info = env.step(action)

    env.reset()
