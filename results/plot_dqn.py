import re, csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = "/home/robotics/tb3_maze_results/dqn_agent.log"
OUT = "/home/robotics/tb3_maze_results"
pat = re.compile(r"Episode:\s*(\d+)\s+score:\s*(-?[\d.]+)\s+memory length:\s*(\d+)\s+epsilon:\s*([\d.]+)")
data = {}
for line in open(LOG, errors="ignore"):
    m = pat.search(line)
    if m:
        data[int(m.group(1))] = (float(m.group(2)), float(m.group(4)))
eps = sorted(data)
scores = [data[e][0] for e in eps]
epsilon = [data[e][1] for e in eps]
print("episodes parsed:", len(eps))
with open(OUT + "/dqn_rewards.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["episode", "score", "epsilon"])
    for e in eps:
        w.writerow([e, data[e][0], data[e][1]])
k = 5
ma = []
for i in range(len(scores)):
    chunk = scores[max(0, i - k + 1):i + 1]
    ma.append(sum(chunk) / len(chunk))
fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
a1.plot(eps, scores, marker="o", ms=3, lw=1, alpha=0.6, label="Episode score")
a1.plot(eps, ma, lw=2.5, label="5-episode moving average")
a1.axhline(0, color="grey", lw=0.8, ls="--")
a1.set_ylabel("Episode reward")
a1.set_title("DQN training: TurtleBot3 Burger in the maze")
a1.legend()
a1.grid(alpha=0.3)
a2.plot(eps, epsilon, color="tab:red", lw=2)
a2.set_ylabel("Epsilon (exploration)")
a2.set_xlabel("Episode")
a2.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(OUT + "/dqn_reward_curve.png", dpi=150)
print("saved curve")
n = min(10, len(scores))
print("mean first", n, ":", round(sum(scores[:n]) / n, 1))
print("mean last", n, ":", round(sum(scores[-n:]) / n, 1))
print("best:", round(max(scores), 1), "worst:", round(min(scores), 1))
print("final epsilon:", round(epsilon[-1], 3))
