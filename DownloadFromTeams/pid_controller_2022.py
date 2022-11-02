
import numpy as np
import gym


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


env = gym.make('CartPole-v1', render_mode='human')
desired_state = np.array([0, 0, 0, 0])
desired_mask = np.array([0, 0, 1, 0])

# Opdracht: Tune de parameter voor de PID controller.
P = 1
I = 0
D = 2

for i_episode in range(20):
    state, truncated = env.reset()
    integral = 0
    derivative = 0
    prev_error = 0
    for t in range(500):
        env.render()
        error = state - desired_state

        integral += error
        derivative = error - prev_error
        prev_error = error

        pid = np.dot(P * error + I * integral + D * derivative, desired_mask)
        action = sigmoid(pid)
        action = np.round(action).astype(np.int32)

        state, reward, done, truncated, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()
