import numpy as np
from conversions import clip, interpolate

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIDController():
  def __init__(self, k_p, k_i, k_d=0., k_f=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d
    self.k_f = k_f  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.unwind_rate = 0.3 / rate
    self.rate = 1.0 / rate
    self.sat_limit = sat_limit

    self.reset()

  @property
  def k_p(self):
    return interpolate(self.speed, self._k_p[0], self._k_p[1]) if isinstance(self.k_p, list) else self._k_p

  @property
  def k_i(self):
    return interpolate(self.speed, self._k_i[0], self._k_i[1]) if isinstance(self.k_i, list) else self._k_i

  @property
  def k_d(self):
    return self.k_d

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.last_measurement = 0.0

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    i = self.i + error * self.k_i * self.rate
    self.d = self.k_d * (measurement - self.last_measurement) / self.rate
    self.f = feedforward * self.k_f
    control = self.p + self.f + i - self.d

    # Update when changing i will move the control away from the limits
    # or when i will move towards the sign of the error
    if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
        (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
        not freeze_integrator:
      self.i = i

    control = self.p + self.f + self.i - self.d

    self.saturated = self._check_saturation(control, check_saturation, error)
    self.last_measurement = measurement
    self.control = clip(control, self.neg_limit, self.pos_limit)
    
    return self.control