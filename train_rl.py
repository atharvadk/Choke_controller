#!/usr/bin/env python3
"""
RL Training Script for Autonomous Choke Control System.

Trains a continuous Actor-Critic / PPO Reinforcement Learning Policy using Gym environment.
Domain Randomization: Randomizes oil rate targets (80-150 bbl/hr) and separator pressure (15-25 bar) during training.
Saves trained policy weights to models/rl_choke_policy.npz and plot to plots/rl_training_curve.png.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from controllers.gym_env import ChokeControlEnv

class ActorCriticPolicy:
    """
    2-Layer Neural Network Actor-Critic Policy (NumPy Implementation).
    Inputs: 7-dim normalized state vector.
    Outputs: Action mean mu in [0, 100]% and State Value V(s).
    """
    def __init__(self, state_dim=7, hidden_dim=32, lr_actor=0.002, lr_critic=0.005):
        self.state_dim = state_dim
        self.hidden_dim = hidden_dim
        self.lr_actor = lr_actor
        self.lr_critic = lr_critic

        # He / Xavier Weight Initialization
        np.random.seed(42)
        self.W1_a = np.random.randn(state_dim, hidden_dim) * np.sqrt(2.0 / state_dim)
        self.b1_a = np.zeros(hidden_dim)
        self.W2_a = np.random.randn(hidden_dim, 1) * np.sqrt(2.0 / hidden_dim)
        self.b2_a = np.zeros(1)

        self.W1_c = np.random.randn(state_dim, hidden_dim) * np.sqrt(2.0 / state_dim)
        self.b1_c = np.zeros(hidden_dim)
        self.W2_c = np.random.randn(hidden_dim, 1) * np.sqrt(2.0 / hidden_dim)
        self.b2_c = np.zeros(1)
        
        self.log_std = 0.5

    def forward_actor(self, s):
        # Hidden layer with ReLU
        h = np.maximum(0, np.dot(s, self.W1_a) + self.b1_a)
        # Output layer with Sigmoid -> scaled to [0, 100]%
        raw = np.dot(h, self.W2_a) + self.b2_a
        mu = 100.0 / (1.0 + np.exp(-np.clip(raw, -10.0, 10.0)))
        return mu[0], h, raw[0]

    def forward_critic(self, s):
        h = np.maximum(0, np.dot(s, self.W1_c) + self.b1_c)
        val = np.dot(h, self.W2_c) + self.b2_c
        return val[0], h

    def select_action(self, s, explore=True):
        mu, _, _ = self.forward_actor(s)
        if explore:
            std = np.exp(self.log_std)
            action = mu + np.random.normal(0, std)
            action = np.clip(action, 0.0, 100.0)
        else:
            action = mu
        return float(action)

    def train_step(self, states, actions, rewards, next_states, dones, gamma=0.98):
        states = np.array(states)
        actions = np.array(actions)
        rewards = np.array(rewards)
        next_states = np.array(next_states)
        dones = np.array(dones)

        N = len(states)
        
        # 1. Compute Temporal Difference Targets & Advantages
        values = np.array([self.forward_critic(s)[0] for s in states])
        next_values = np.array([self.forward_critic(ns)[0] for ns in next_states])
        
        targets = rewards + gamma * next_values * (1.0 - dones)
        advantages = targets - values
        
        # Normalize advantages
        if np.std(advantages) > 1e-8:
            advantages = (advantages - np.mean(advantages)) / (np.std(advantages) + 1e-8)

        # 2. Update Critic (Mean Squared Error Loss)
        for i in range(N):
            s = states[i]
            target = targets[i]
            val, h_c = self.forward_critic(s)
            error = val - target
            
            # Backpropagation
            dW2_c = h_c[:, None] * error
            db2_c = error
            dh_c = error * self.W2_c.ravel()
            dh_c[h_c <= 0] = 0  # ReLU derivative
            dW1_c = np.outer(s, dh_c)
            db1_c = dh_c

            self.W2_c -= self.lr_critic * dW2_c
            self.b2_c -= self.lr_critic * db2_c
            self.W1_c -= self.lr_critic * dW1_c
            self.b1_c -= self.lr_critic * db1_c

        # 3. Update Actor (Policy Gradient)
        std = np.exp(self.log_std)
        for i in range(N):
            s = states[i]
            a = actions[i]
            adv = advantages[i]
            
            mu, h_a, raw = self.forward_actor(s)
            
            # Derivative of mu w.r.t raw sigmoid
            sig = mu / 100.0
            dsig = sig * (1.0 - sig) * 100.0
            
            # Policy gradient: d/d_mu log P(a|mu) = (a - mu) / std^2
            grad_mu = (a - mu) / (std ** 2 + 1e-8) * adv
            grad_raw = grad_mu * dsig
            
            dW2_a = h_a[:, None] * grad_raw
            db2_a = grad_raw
            dh_a = grad_raw * self.W2_a.ravel()
            dh_a[h_a <= 0] = 0
            dW1_a = np.outer(s, dh_a)
            db1_a = dh_a

            self.W2_a += self.lr_actor * dW2_a
            self.b2_a += self.lr_actor * db2_a
            self.W1_a += self.lr_actor * dW1_a
            self.b1_a += self.lr_actor * db1_a

    def save(self, filepath):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        np.savez(
            filepath,
            W1_a=self.W1_a, b1_a=self.b1_a, W2_a=self.W2_a, b2_a=self.b2_a,
            W1_c=self.W1_c, b1_c=self.b1_c, W2_c=self.W2_c, b2_c=self.b2_c,
            log_std=self.log_std
        )

    def load(self, filepath):
        data = np.load(filepath)
        self.W1_a = data['W1_a']
        self.b1_a = data['b1_a']
        self.W2_a = data['W2_a']
        self.b2_a = data['b2_a']
        self.W1_c = data['W1_c']
        self.b1_c = data['b1_c']
        self.W2_c = data['W2_c']
        self.b2_c = data['b2_c']
        self.log_std = float(data['log_std'])

def main():
    print("==========================================================================")
    print("   AUTONOMOUS CHOKE CONTROL SYSTEM: REINFORCEMENT LEARNING TRAINING      ")
    print("==========================================================================")

    env = ChokeControlEnv(config_path="configs/default.yaml", dt=5.0, max_steps=200)
    policy = ActorCriticPolicy(state_dim=7, hidden_dim=32, lr_actor=0.001, lr_critic=0.003)

    num_episodes = 80
    episode_returns = []

    for ep in range(1, num_episodes + 1):
        # Domain Randomization: Randomize target oil rate and separator pressure
        target_oil = np.random.uniform(90.0, 140.0)
        p_sep = np.random.uniform(16.0, 24.0)
        
        env.target_oil_bbl_hr = target_oil
        obs, info = env.reset()
        env.sim.state.separator_pressure = p_sep
        
        states, actions, rewards, next_states, dones = [], [], [], [], []
        total_reward = 0.0

        for t in range(200):
            action = policy.select_action(obs, explore=True)
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            states.append(obs)
            actions.append(action)
            rewards.append(reward)
            next_states.append(next_obs)
            dones.append(done)

            obs = next_obs
            total_reward += reward
            if done:
                break

        # Train policy on episode trajectory
        policy.train_step(states, actions, rewards, next_states, dones)
        
        # Decay exploration noise
        policy.log_std = max(-1.5, policy.log_std - 0.01)
        episode_returns.append(total_reward)

        if ep % 10 == 0:
            avg_ret = np.mean(episode_returns[-10:])
            print(f"  Episode {ep:3d}/{num_episodes} -> Target: {target_oil:5.1f} bbl/hr | Return: {total_reward:8.2f} (10-ep avg: {avg_ret:8.2f})")

    model_path = "models/rl_choke_policy.npz"
    policy.save(model_path)
    print(f"\nSaved trained RL policy weights to {model_path}.")

    # Generate Learning Curve Plot
    os.makedirs("plots", exist_ok=True)
    plt.figure(figsize=(9, 5))
    plt.plot(range(1, num_episodes + 1), episode_returns, 'b-', alpha=0.4, label='Episode Return')
    # Rolling average
    smooth_returns = pd.Series(episode_returns).rolling(window=5, min_periods=1).mean()
    plt.plot(range(1, num_episodes + 1), smooth_returns, 'b-', linewidth=2.5, label='5-Ep Moving Avg')
    plt.title("Reinforcement Learning (Actor-Critic) Training Learning Curve")
    plt.xlabel("Training Episode")
    plt.ylabel("Cumulative Episode Return")
    plt.grid(True)
    plt.legend(loc='lower right')
    plt.tight_layout()
    plt.savefig("plots/rl_training_curve.png", dpi=150)
    plt.close()
    print("Saved training curve plot to plots/rl_training_curve.png.")

if __name__ == "__main__":
    import pandas as pd
    main()
