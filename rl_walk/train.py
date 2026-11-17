"""Train PPO on GoalEnv and compare it with the map-based greedy baseline.

python rl_walk/train.py --steps 1000000            # train, save rl_walk/runs/ppo_goal.zip, evaluate
python rl_walk/train.py --eval-only                # evaluate a saved policy and the baseline
Needs gymnasium, stable-baselines3 and torch in the project's .venv (pip install gymnasium stable-baselines3 torch).

The evaluation runs in the same simulator the policy was trained in (plus STRESS plants outside its ranges): it shows
that training worked and how much margin there is, not how the robot will do. The greedy baseline looks one 0.6 s
arc ahead on the nominal map, delay and tick; it is not the lab's MPC.
"""
import argparse
import os

import numpy as np

from env import GoalEnv, GreedyMap

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "runs")


# Plants outside the training ranges (env.RANDOM): what the policy was never shown.
STRESS = {
    "dead time 0.35-0.45 s": dict(delay=(0.35, 0.45)),
    "control tick 0.20-0.25 s": dict(tick=(0.20, 0.25)),
    "map error per cell x2": dict(cell_err=(2.0, 2.0)),
    "speed x0.70-0.80 (weak battery)": dict(gv=(0.70, 0.80)),
    "turn rate x1.3-1.5": dict(gw=(1.3, 1.5)),
    "turn bias +-0.3 rad/s": dict(wbias=(0.3, 0.3)),
    "noise x2": dict(noise=(2.0, 2.0)),
}


def evaluate(act, randomize, n=200, seed=1000, ranges=None):
    """Success rate, time to goal of the successes, share of episodes that left the arena."""
    env = GoalEnv(randomize=randomize, ranges=ranges)
    ok, out, times = 0, 0, []
    for i in range(n):
        obs, _ = env.reset(seed=seed + i)
        while True:
            obs, r, term, trunc, info = env.step(act(env, obs))
            if term or trunc:
                break
        ok += info["reached"]
        out += info["out"]
        if info["reached"]:
            times.append(info["t"])
    med = np.median(times) if times else float("nan")
    return f"reached {100 * ok / n:5.1f} %, left arena {100 * out / n:4.1f} %, median time {med:5.1f} s"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--n-eval", type=int, default=200)
    ap.add_argument("--policy", help="--eval-only: a saved policy other than runs/ppo_goal.zip")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "ppo_goal.zip")
    if not a.eval_only:
        venv = make_vec_env(lambda: GoalEnv(randomize=True), n_envs=a.envs, vec_env_cls=SubprocVecEnv, seed=0)
        model = PPO("MlpPolicy", venv, n_steps=512, batch_size=1024, gamma=0.99, gae_lambda=0.95,
                    ent_coef=0.01, learning_rate=3e-4, policy_kwargs=dict(net_arch=[64, 64]), verbose=1,
                    device="cpu", seed=0)
        model.learn(total_timesteps=a.steps)
        model.save(path)
    model = PPO.load(a.policy or path, device="cpu")

    ppo = lambda env, obs: int(model.predict(obs, deterministic=True)[0])
    greedy = GreedyMap()
    base = lambda env, obs: greedy(env)
    for randomize in (False, True):
        tag = "randomised" if randomize else "nominal   "
        print(f"{tag} PPO     : {evaluate(ppo, randomize, a.n_eval)}")
        print(f"{tag} greedy  : {evaluate(base, randomize, a.n_eval)}")
    print("outside the training ranges (everything else randomised as in training):")
    for name, ranges in STRESS.items():
        print(f"  {name:32s} PPO   : {evaluate(ppo, True, a.n_eval, ranges=ranges)}")
        print(f"  {'':32s} greedy: {evaluate(base, True, a.n_eval, ranges=ranges)}")


if __name__ == "__main__":
    main()
