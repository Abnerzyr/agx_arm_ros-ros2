#!/usr/bin/env python3

"""Grasp residual-refinement RL module.

A one-step contextual-bandit policy that refines the CNN grasp proposal.
Runs in-process inside yolo_grasp_node. Reward comes from the executor's
`grasp_result` topic (0 = not executed / failed, 1 = empty, 2 = held).
"""

import json
import os
import time
from collections import deque

import numpy as np
import torch
import torch.nn as nn

from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import Int32
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool


def build_patch(depth, rgb, u, v, size, z):
    """Local RGB-D patch centered on (u, v), normalized for the policy.

    depth: float meters (H, W); rgb: BGR (H, W, 3); u/v: sub-pixel floats.
    Depth channel normalized by the grasp depth z, RGB scaled to [0, 1].
    Returns (size, size, 4) float32 array.
    """
    size = int(size)
    half = size // 2
    H, W = depth.shape
    ui = int(round(u))
    vi = int(round(v))
    y0 = max(0, vi - half)
    x0 = max(0, ui - half)
    y1 = min(H, vi - half + size)
    x1 = min(W, ui - half + size)
    patch = np.zeros((size, size, 4), np.float32)
    hh = y1 - y0
    ww = x1 - x0
    if hh > 0 and ww > 0:
        dy0 = y0 - (vi - half)
        dx0 = x0 - (ui - half)
        patch[dy0:dy0 + hh, dx0:dx0 + ww, :3] = \
            rgb[y0:y1, x0:x1].astype(np.float32) / 255.0
        dp = depth[y0:y1, x0:x1].astype(np.float32)
        ref = float(z) if (z is not None and z > 0.05) else \
            float(np.median(dp[dp > 0.05])) if np.any(dp > 0.05) else 1.0
        d_norm = np.full((hh, ww), 1.0, np.float32)
        if np.isfinite(ref) and ref > 1e-3:
            ok = np.isfinite(dp) & (dp > 0.05)
            d_norm = np.where(ok, dp / ref, 1.0)
        patch[dy0:dy0 + hh, dx0:dx0 + ww, 3] = np.clip(d_norm, 0.0, 3.0)
    return patch


class _GraspQNet(nn.Module):
    def __init__(self, n_actions=13, n_scalars=4):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(4, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
        )
        self.head = nn.Sequential(
            nn.Linear(64 + n_scalars, 64), nn.ReLU(),
            nn.Linear(64, n_actions),
        )

    def forward(self, x, s):
        f = self.conv(x).flatten(1)
        return self.head(torch.cat([f, s], dim=1))


class GraspRLRefiner:
    N_ACTIONS = 13  # no-op(1) + 8-dir shift + angle +- + depth +-

    # action layout
    _DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (1, -1), (-1, 1), (1, 1)]

    def __init__(self, node, cfg):
        self.node = node
        self.log = node.get_logger()
        self.patch_size = int(cfg['patch_size'])
        self.pixel_step = float(cfg['pixel_step'])
        self.angle_step = float(cfg['angle_step_deg']) * np.pi / 180.0
        self.depth_step = float(cfg['depth_step'])
        self.eps_start = float(cfg['epsilon_start'])
        self.eps_end = float(cfg['epsilon_end'])
        self.eps_decay = float(cfg['epsilon_decay'])
        self.replay_capacity = int(cfg['replay_capacity'])
        self.batch_size = int(cfg['batch_size'])
        self.grad_steps = int(cfg['grad_steps'])
        self.lr = float(cfg['lr'])
        self.ckpt_interval = int(cfg['checkpoint_interval'])
        self.inflight_timeout = float(cfg['inflight_timeout'])
        self.r_success = float(cfg['reward_success'])
        self.r_empty = float(cfg['reward_empty'])
        self.r_fail = float(cfg['reward_fail'])
        self.stats_interval = int(cfg['stats_interval'])
        self.n_scalars = int(cfg['n_scalars'])
        self.ckpt_dir = cfg['checkpoint_dir']
        self.log_dir = cfg['log_dir']

        self.net = _GraspQNet(self.N_ACTIONS, self.n_scalars)
        with torch.no_grad():
            self.net.head[-1].bias[0] += 0.1  # warm start toward no-op
        self.opt = torch.optim.Adam(self.net.parameters(), lr=self.lr)
        self.step = 0
        self.eps = self.eps_start
        self.training = False
        self.replay = deque(maxlen=self.replay_capacity)
        self._last = None          # (patch, scalars, action) latest sampled
        self._inflight = None      # dict of the attempt being executed
        self._abort_pending = False
        self._last_layer = 0
        self._stats = {'n': 0, 'succ': 0, 'empty': 0, 'fail': 0}

        for d in (self.ckpt_dir, self.log_dir):
            os.makedirs(d, exist_ok=True)
        self._load()

        node.create_service(
            SetBool, 'grasp_rl/set_training', self._set_training_cb)
        node.create_service(EmptySrv, 'grasp_rl/abort', self._abort_cb)
        node.create_subscription(
            Int32, 'task_command', self._task_cmd_cb, 10)
        node.create_subscription(
            Int32, 'grasp_result', self._grasp_result_cb, 10)
        node.create_subscription(
            EmptyMsg, 'manual_grasp_start', self._grasp_start_cb, 10)
        node.create_timer(1.0, self._tick)
        self.log.info('[RL] grasp refiner ready '
                      f'(checkpoint={self.ckpt_dir})')

    # ------------------------------------------------------------------
    # policy interface (called by yolo_grasp_node)
    # ------------------------------------------------------------------
    def observe(self, patch_rgbd, scalars):
        """Feed the current state, sample an action, remember it as last."""
        patch = np.asarray(patch_rgbd, np.float32)
        scal = np.asarray(scalars, np.float32)
        eps_eff = self.eps if self.training else 0.0
        if np.random.rand() > eps_eff:
            action = self._greedy(patch, scal)
        else:
            action = int(np.random.randint(self.N_ACTIONS))
        self._last = (patch, scal, action)
        return action

    def apply_action(self, action, u, v, angle, w, h):
        """Map action index to (u', v', angle', dz)."""
        du = dv = 0.0
        dangle = 0.0
        dz = 0.0
        if 1 <= action <= 8:
            du_, dv_ = self._DIRS[action - 1]
            du = du_ * self.pixel_step
            dv = dv_ * self.pixel_step
        elif action == 9:
            dangle = self.angle_step
        elif action == 10:
            dangle = -self.angle_step
        elif action == 11:
            dz = -self.depth_step
        elif action == 12:
            dz = self.depth_step
        u2 = min(max(u + du, 0.0), float(w - 1))
        v2 = min(max(v + dv, 0.0), float(h - 1))
        return u2, v2, angle + dangle, dz

    def is_training(self):
        return self.training

    def save_checkpoint(self):
        path = os.path.join(self.ckpt_dir, 'policy.pt')
        torch.save({
            'net': self.net.state_dict(),
            'opt': self.opt.state_dict(),
            'step': self.step,
            'eps': self.eps,
            'n_scalars': self.n_scalars,
        }, path)
        self.log.info(f'[RL] checkpoint saved (step={self.step}, '
                      f'eps={self.eps:.3f}) -> {path}')

    # ------------------------------------------------------------------
    # reward / learning
    # ------------------------------------------------------------------
    def _reward(self, code):
        if code == 2:
            return self.r_success
        if code == 1:
            return self.r_empty
        return self.r_fail

    def _grasp_start_cb(self, msg):
        del msg
        if self._last is None:
            self._inflight = None
            return
        self._inflight = {
            'patch': self._last[0],
            'scalars': self._last[1],
            'action': self._last[2],
            't': time.time(),
            'layer': self._last_layer,
            'external_abort': self._abort_pending,
        }
        self._abort_pending = False
        self.log.info(
            f'[RL] inflight snapshot action={self._inflight["action"]} '
            f'layer={self._inflight["layer"]} '
            f'abort={self._inflight["external_abort"]}')

    def _grasp_result_cb(self, msg):
        code = int(msg.data)
        if self._inflight is None:
            self.log.warn(
                f'[RL] grasp_result={code} with no inflight; ignored')
            return
        if self._inflight['external_abort']:
            reward = 0.0
            self.log.info(
                '[RL] external abort; sample logged but not learned')
        else:
            reward = self._reward(code)
        self._log_sample(code, reward)
        if self.training and not self._inflight['external_abort']:
            self.replay.append(
                (self._inflight['patch'], self._inflight['scalars'],
                 self._inflight['action'], reward))
            self._update()
            self.eps = max(self.eps_end, self.eps * self.eps_decay)
        self.step += 1
        self._update_stats(code, self._inflight['external_abort'])
        if self.step % self.ckpt_interval == 0:
            self.save_checkpoint()
        self._inflight = None

    def _update(self):
        if len(self.replay) < self.batch_size:
            return
        idx = np.random.choice(len(self.replay), self.batch_size,
                               replace=False)
        batch = [self.replay[i] for i in idx]
        patches = torch.from_numpy(np.stack([b[0] for b in batch])).float()
        patches = patches.permute(0, 3, 1, 2)
        scalars = torch.from_numpy(np.stack([b[1] for b in batch])).float()
        actions = torch.tensor([b[2] for b in batch],
                               dtype=torch.long).unsqueeze(1)
        rewards = torch.tensor([b[3] for b in batch],
                               dtype=torch.float).unsqueeze(1)
        for _ in range(self.grad_steps):
            q = self.net(patches, scalars).gather(1, actions)
            loss = nn.functional.smooth_l1_loss(q, rewards)
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()

    def _greedy(self, patch, scal):
        with torch.no_grad():
            x = torch.from_numpy(patch).float().permute(2, 0, 1).unsqueeze(0)
            s = torch.from_numpy(scal).float().unsqueeze(0)
            q = self.net(x, s)[0]
            return int(q.argmax().item())

    # ------------------------------------------------------------------
    # logging / stats / persistence
    # ------------------------------------------------------------------
    def _log_sample(self, code, reward):
        if self._inflight is None:
            return
        meta = {
            'step': self.step,
            't': time.time(),
            'result': code,
            'reward': reward,
            'action': self._inflight['action'],
            'layer': self._inflight['layer'],
            'external_abort': bool(self._inflight['external_abort']),
            'training': bool(self.training),
        }
        try:
            np.savez(os.path.join(self.log_dir, f'sample_{self.step:06d}.npz'),
                     patch=self._inflight['patch'],
                     scalars=self._inflight['scalars'],
                     action=np.array(meta['action']),
                     reward=np.array(reward),
                     result=np.array(code),
                     external_abort=np.array(meta['external_abort']))
            with open(os.path.join(self.log_dir, 'samples.jsonl'), 'a') as f:
                f.write(json.dumps(meta) + '\n')
        except Exception as exc:
            self.log.warn(f'[RL] sample log failed: {exc}')

    def _update_stats(self, code, external_abort):
        if external_abort:
            return
        self._stats['n'] += 1
        if code == 2:
            self._stats['succ'] += 1
        elif code == 1:
            self._stats['empty'] += 1
        else:
            self._stats['fail'] += 1
        if self._stats['n'] % self.stats_interval == 0:
            s = self._stats
            self.log.info(
                f'[RL] stats: n={s["n"]} succ={s["succ"]} '
                f'empty={s["empty"]} fail={s["fail"]} '
                f'rate={s["succ"] / max(s["n"], 1):.2f} '
                f'eps={self.eps:.3f}')

    def _load(self):
        path = os.path.join(self.ckpt_dir, 'policy.pt')
        if not os.path.exists(path):
            return
        try:
            ck = torch.load(path, map_location='cpu', weights_only=False)
            self.net.load_state_dict(ck['net'])
            self.opt.load_state_dict(ck['opt'])
            self.step = int(ck.get('step', 0))
            self.eps = float(ck.get('eps', self.eps_start))
            self.log.info(
                f'[RL] loaded checkpoint (step={self.step}, '
                f'eps={self.eps:.3f}) from {path}')
        except Exception as exc:
            self.log.warn(f'[RL] checkpoint load failed ({exc}); '
                          'starting fresh')

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _set_training_cb(self, request, response):
        self.training = bool(request.data)
        if not self.training:
            self.save_checkpoint()
        self.log.info(f'[RL] training set to {self.training} '
                      f'(eps={self.eps:.3f})')
        response.success = True
        response.message = f'training={self.training}'
        return response

    def _abort_cb(self, request, response):
        del request
        if self._inflight is not None:
            self._inflight['external_abort'] = True
            self._abort_pending = False
        else:
            self._abort_pending = True
        self.log.warn('[RL] abort marked; in-flight sample will not be '
                      'learned')
        return response

    def _task_cmd_cb(self, msg):
        self._last_layer = int(msg.data)

    def _tick(self):
        if (self._inflight is not None
                and time.time() - self._inflight['t'] > self.inflight_timeout):
            self.log.warn(
                f'[RL] inflight exceeded {self.inflight_timeout:.0f}s; '
                'marking external_abort (will not be learned)')
            self._inflight['external_abort'] = True
