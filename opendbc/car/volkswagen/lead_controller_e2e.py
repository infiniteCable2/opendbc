import cereal.messaging as messaging
import numpy as np

STOP_SPEED_THRESHOLD = 0.5 # m/s

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

    vx = np.array(self.sm['modelV2'].velocity.x)
    vy = np.array(self.sm['modelV2'].velocity.y)
    vz = np.array(self.sm['modelV2'].velocity.z)
    t  = np.array(self.sm['modelV2'].velocity.t)

    if len(t) < 2:
      self.reset()
      return

    v = np.sqrt(vx**2 + vy**2 + vz**2)

    below_thresh = np.where(v < STOP_SPEED_THRESHOLD)[0]

    if len(below_thresh) == 0:
      self.reset()
      return

    stop_idx = below_thresh[0]

    dt = np.diff(t)
    v_mid = 0.5 * (v[:-1] + v[1:])

    valid_len = min(stop_idx, len(dt))
    self.distance = np.sum(v_mid[:valid_len] * dt[:valid_len])
    self.has_lead = True
