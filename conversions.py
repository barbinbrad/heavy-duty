import numpy as np
from math import dist

class Conversions:
    IN_TO_M = 0.0254
    M_TO_IN = 1. / IN_TO_M
    M_TO_FT = M_TO_IN / 12
    GAL_TO_LT = 3.785412534258
    LT_TO_GAL = 1 / GAL_TO_LT
    MPH_TO_KPH = 1.609344
    KPH_TO_MPH = 1. / MPH_TO_KPH
    MS_TO_KPH = 3.6
    KPH_TO_MS = 1. / MS_TO_KPH
    MS_TO_MPH = MS_TO_KPH * KPH_TO_MPH
    MPH_TO_MS = MPH_TO_KPH * KPH_TO_MS
    MS_TO_KNOTS = 1.94384
    KNOTS_TO_MS = 1. / MS_TO_KNOTS
    KNOTS_TO_KPH = KNOTS_TO_MS * MS_TO_KPH
    #Angle
    DEG_TO_RAD = np.pi / 180.
    RAD_TO_DEG = 1. / DEG_TO_RAD
    #Mass
    LB_TO_KG = 0.453592
    KG_TO_LB = 1 / LB_TO_KG

def distance(p, q):
    # p and q are lists
    return dist(p,q)

def clip(x, lo, hi):
  return max(lo, min(hi, x))

def interpolate(x, list_of_x, list_of_y):
  N = len(list_of_x)

  def get_interpolate(xv):
    hi = 0
    while hi < N and xv > list_of_x[hi]:
      hi += 1
    low = hi - 1
    return list_of_y[-1] if hi == N and xv > list_of_x[low] else (
      list_of_y[0] if hi == 0 else
      (xv - list_of_x[low]) * (list_of_y[hi] - list_of_y[low]) / (list_of_x[hi] - list_of_x[low]) + list_of_y[low])

  return [get_interpolate(v) for v in x] if hasattr(x, '__iter__') else get_interpolate(x)

def mean(x):
  return sum(x) / len(x)