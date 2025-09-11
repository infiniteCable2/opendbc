import cereal.messaging as messaging
import numpy as np

STOP_SPEED_THRESHOLD = 0.5  # m/s

class LeadControllerE2E():
  def __init__(self):
    self.sm = messaging.SubMaster(['modelV2'])
    self.reset()

  def reset(self):
    self.distance = np.inf
    self.has_lead = False

  def is_lead_present(self):
    return self.has_lead

  def get_distance(self):
    return self.distance

  def update(self):
    self.sm.update(0)
    if not self.sm.updated['modelV2']:
      return

    model = self.sm['modelV2'].velocity

    if len(model.t) < 2:
      self.reset()
      return

    v_all = np.stack([model.x, model.y, model.z], axis=1)
    v = np.linalg.norm(v_all, axis=1)

    below_thresh = np.where(v < STOP_SPEED_THRESHOLD)[0]
    if len(below_thresh) == 0:
      self.reset()
      return

    stop_idx = below_thresh[0]

    t = np.array(model.t)
    dt = np.diff(t)

    v_mid = 0.5 * (v[:-1] + v[1:])
    valid_len = min(stop_idx, len(dt))
    self.distance = max(1, np.sum(v_mid[:valid_len] * dt[:valid_len]))
    self.has_lead = True
