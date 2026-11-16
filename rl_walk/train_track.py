"""Train PPO on TrackEnv (reference tracking) and compare it with the greedy map-based tracker.

python rl_walk/train_track.py --shapes circle --name circle --switch-cost 0.05    # stage 1: circles only
python rl_walk/train_track.py --shapes circle,figure8 --name track --init circle  # stage 2: + figure-8s, from stage 1
python rl_walk/train_track.py --eval-only --name track

Policies go to rl_walk/runs/ppo_<name>.zip: ppo_circle (stage 1), ppo_track (stage 2, 1.5M steps, switch cost 0.3) and
ppo_track_calm (the same with switch cost 1.0: fewer command changes, about 0.5 cm more error in sim). As in train.py, the evaluation runs in the simulator the policy was
trained in: it shows that training worked, not how the robot will do. Errors are counted after the first 3 s.
"""
import argparse
import os

import numpy as np

from track_env import GreedyTrack, TrackEnv, benchmark

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "runs")
STRESS = {
    "dead time 0.35-0.45 s": dict(delay=(0.35, 0.45)),
    "map error per cell x2": dict(cell_err=(2.0, 2.0)),
    "speed x0.70-0.80": dict(gv=(0.70, 0.80)),
    "noise x2": dict(noise=(2.0, 2.0)),
}


def evaluate(act, n=100, seed=1000, **env_kw):
    """Over n episodes: median of the episodes' median and rms errors, the worst error, the share of episodes that
    lost the reference, and how often the command changes."""
    env = TrackEnv(**env_kw)
    med, rms, worst, lost, changes, ticks = [], [], 0.0, 0, 0, 0
    for i in range(n):
        obs, _ = env.reset(seed=seed + i)
        errs, prev = [], None
        while True:
            a = act(env, obs)
            changes += prev is not None and a != prev
            prev = a
            ticks += 1
            obs, r, term, trunc, info = env.step(a)
            if info["t"] > 3.0:
                errs.append(info["err"])
            if term or trunc:
                break
        lost += info["lost"]
        if errs:
            med.append(np.median(errs))
            rms.append(np.sqrt(np.mean(np.square(errs))))
            worst = max(worst, max(errs))
    return (f"median {100 * np.median(med):4.1f} cm, rms {100 * np.median(rms):4.1f} cm, worst {100 * worst:4.1f} cm, "
            f"lost {100 * lost / n:4.1f} %, command changes on {100 * changes / ticks:2.0f} % of ticks")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shapes", default="circle,figure8")
    ap.add_argument("--name", default="track")
    ap.add_argument("--init", help="continue from runs/ppo_<init>.zip")
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--eval-only", action="store_true")
    ap.add_argument("--n-eval", type=int, default=100)
    ap.add_argument("--switch-cost", type=float, default=0.3, help="reward taken off when the command changes")
    a = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.vec_env import SubprocVecEnv

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"ppo_{a.name}.zip")
    shapes = tuple(a.shapes.split(","))
    if not a.eval_only:
        venv = make_vec_env(lambda: TrackEnv(shapes=shapes, switch_cost=a.switch_cost), n_envs=a.envs, vec_env_cls=SubprocVecEnv, seed=0)
        kw = dict(n_steps=512, batch_size=1024, gamma=0.99, gae_lambda=0.95, ent_coef=0.01, learning_rate=3e-4,
                  device="cpu")
        if a.init:
            model = PPO.load(os.path.join(OUT, f"ppo_{a.init}.zip"), env=venv, **kw)
        else:
            model = PPO("MlpPolicy", venv, policy_kwargs=dict(net_arch=[64, 64]), verbose=1, seed=0, **kw)
        model.learn(total_timesteps=a.steps)
        model.save(path)
    model = PPO.load(path, device="cpu")

    ppo = lambda env, obs: int(model.predict(obs, deterministic=True)[0])
    greedy = GreedyTrack()
    base = lambda env, obs: greedy(env)
    cases = {"random circles": dict(shapes=("circle",)), "random figure-8s": dict(shapes=("figure8",)),
             "benchmark circle (r 15 cm, 20 s)": dict(ref=benchmark("circle")),
             "benchmark figure-8 (50 x 30 cm, 40 s)": dict(ref=benchmark("figure8"))}
    print(f"policy {path}; plant randomised as in training")
    for name, kw in cases.items():
        print(f"  {name:38s} PPO   : {evaluate(ppo, a.n_eval, **kw)}")
        print(f"  {'':38s} greedy: {evaluate(base, a.n_eval, **kw)}")
    print("benchmark figure-8, plant at its nominal values:")
    print(f"  {'':38s} PPO   : {evaluate(ppo, a.n_eval, ref=benchmark('figure8'), randomize=False)}")
    print(f"  {'':38s} greedy: {evaluate(base, a.n_eval, ref=benchmark('figure8'), randomize=False)}")
    print("benchmark figure-8, outside the training ranges:")
    for name, ranges in STRESS.items():
        print(f"  {name:38s} PPO   : {evaluate(ppo, a.n_eval, ref=benchmark('figure8'), ranges=ranges)}")
        print(f"  {'':38s} greedy: {evaluate(base, a.n_eval, ref=benchmark('figure8'), ranges=ranges)}")


if __name__ == "__main__":
    main()
