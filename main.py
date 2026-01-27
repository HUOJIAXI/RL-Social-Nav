import gymnasium as gym
import ale_py
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.atari_wrappers import AtariWrapper
from stable_baselines3.common.vec_env import VecFrameStack, DummyVecEnv

gym.register_envs(ale_py)


class TrainingHistoryCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.history = {
            "timesteps": [],
            "value_loss": [],
            "policy_loss": [],
            "entropy_loss": [],
            "approx_kl": [],
            "clip_fraction": [],
            "explained_variance": [],
            "episode_rewards": [],
            "episode_lengths": [],
        }

    def _on_step(self) -> bool:
        return True

    def _on_rollout_end(self) -> None:
        # Record metrics after each rollout
        self.history["timesteps"].append(self.num_timesteps)

        if len(self.model.logger.name_to_value) > 0:
            logs = self.model.logger.name_to_value
            self.history["value_loss"].append(logs.get("train/value_loss", np.nan))
            self.history["policy_loss"].append(logs.get("train/policy_gradient_loss", np.nan))
            self.history["entropy_loss"].append(logs.get("train/entropy_loss", np.nan))
            self.history["approx_kl"].append(logs.get("train/approx_kl", np.nan))
            self.history["clip_fraction"].append(logs.get("train/clip_fraction", np.nan))
            self.history["explained_variance"].append(logs.get("train/explained_variance", np.nan))

        # Record episode stats if available
        if len(self.model.ep_info_buffer) > 0:
            ep_rewards = [ep["r"] for ep in self.model.ep_info_buffer]
            ep_lengths = [ep["l"] for ep in self.model.ep_info_buffer]
            self.history["episode_rewards"].append(np.mean(ep_rewards))
            self.history["episode_lengths"].append(np.mean(ep_lengths))


def plot_training_history(history):
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle("PPO Training History - Breakout", fontsize=14)

    # Value Loss
    if history["value_loss"]:
        axes[0, 0].plot(history["timesteps"][:len(history["value_loss"])], history["value_loss"])
        axes[0, 0].set_title("Value Loss")
        axes[0, 0].set_xlabel("Timesteps")
        axes[0, 0].set_ylabel("Loss")

    # Policy Loss
    if history["policy_loss"]:
        axes[0, 1].plot(history["timesteps"][:len(history["policy_loss"])], history["policy_loss"])
        axes[0, 1].set_title("Policy Gradient Loss")
        axes[0, 1].set_xlabel("Timesteps")
        axes[0, 1].set_ylabel("Loss")

    # Entropy Loss
    if history["entropy_loss"]:
        axes[0, 2].plot(history["timesteps"][:len(history["entropy_loss"])], history["entropy_loss"])
        axes[0, 2].set_title("Entropy Loss")
        axes[0, 2].set_xlabel("Timesteps")
        axes[0, 2].set_ylabel("Loss")

    # Episode Rewards
    if history["episode_rewards"]:
        axes[1, 0].plot(history["episode_rewards"])
        axes[1, 0].set_title("Episode Rewards (Mean)")
        axes[1, 0].set_xlabel("Rollout")
        axes[1, 0].set_ylabel("Reward")

    # Explained Variance
    if history["explained_variance"]:
        axes[1, 1].plot(history["timesteps"][:len(history["explained_variance"])], history["explained_variance"])
        axes[1, 1].set_title("Explained Variance")
        axes[1, 1].set_xlabel("Timesteps")
        axes[1, 1].set_ylabel("Variance")

    # KL Divergence
    if history["approx_kl"]:
        axes[1, 2].plot(history["timesteps"][:len(history["approx_kl"])], history["approx_kl"])
        axes[1, 2].set_title("Approximate KL Divergence")
        axes[1, 2].set_xlabel("Timesteps")
        axes[1, 2].set_ylabel("KL")

    plt.tight_layout()
    plt.savefig("training_history.png", dpi=150)
    plt.show()
    print("Training history saved to training_history.png")


# Create Atari environment (GPU beneficial with CnnPolicy)
env = gym.make("ALE/Breakout-v5")
env = AtariWrapper(env)
env = DummyVecEnv([lambda: env])
env = VecFrameStack(env, n_stack=4)

# Create callback to record history
history_callback = TrainingHistoryCallback()

model = PPO("CnnPolicy", env, verbose=1, device="cuda")
model.learn(total_timesteps=2_000_000, callback=history_callback)

# Plot training history
plot_training_history(history_callback.history)

# Evaluate trained model
vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render()

env.close()
