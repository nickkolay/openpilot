#!/usr/bin/env python3
from dataclasses import dataclass
import os
import argparse

import matplotlib.pyplot as plt

from tools.lib.logreader import logreader_from_route_or_segment


@dataclass
class LateralMotionControl2:
  ramp_type: int
  precision: int
  mode: int
  hands_off: int
  checksum: int
  curvature: int
  path_angle: int
  path_offset: int
  curvature_rate: int
  counter: int

  @classmethod
  def from_can(cls, dat):
    ramp_type = dat[0] & 0x3
    precision = (dat[0] >> 2) & 0x3
    mode = (dat[0] >> 4) & 0x7
    hands_off = (dat[0] >> 7) & 0x1
    checksum = dat[1]
    curvature = (dat[2] << 3) | (dat[3] >> 5)
    path_angle = ((dat[3] & 0x1f) << 6) | (dat[4] >> 2)
    path_offset = ((dat[4] & 0x3) << 8) | dat[5]
    curvature_rate = (dat[6] << 3) | (dat[7] >> 5)
    counter = (dat[7] & 0x1e) >> 1
    return cls(ramp_type, precision, mode, hands_off, checksum, curvature, path_angle, path_offset, curvature_rate, counter)

  @property
  def alt_checksum(self):
    return 0xFF - (self.ramp_type + self.precision + self.mode + self.hands_off + self.curvature + self.path_angle + self.path_offset + self.curvature_rate + self.counter) & 0xFF


def update(msg):
  if msg.which() == "can":
    for canmsg in msg.can:
      if canmsg.address != 982:
        continue

      yield LateralMotionControl2.from_can(canmsg.dat)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Steering accuracy measurement tool')
  parser.add_argument('route', type=str, nargs='?', help='route name')
  parser.add_argument('--cache', default=True, action='store_true', help="use cached data, default to True")
  args = parser.parse_args()

  if args.cache:
    os.environ['FILEREADER_CACHE'] = '1'

  print(f"loading {args.route}...")
  lr = logreader_from_route_or_segment(args.route, sort_by_time=True)

  ramp_type, precision, mode, hands_off, checksum, curvature, path_angle, path_offset, curvature_rate, counter = [], [], [], [], [], [], [], [], [], []
  alt_checksum = []
  for msg in lr:
    if msg.which() != 'can':
      continue

    for lat_ctl in update(msg):
      ramp_type.append(lat_ctl.ramp_type)
      precision.append(lat_ctl.precision)
      mode.append(lat_ctl.mode)
      hands_off.append(lat_ctl.hands_off)
      checksum.append(lat_ctl.checksum)
      alt_checksum.append(lat_ctl.alt_checksum)
      curvature.append(lat_ctl.curvature)
      path_angle.append(lat_ctl.path_angle)
      path_offset.append(lat_ctl.path_offset)
      curvature_rate.append(lat_ctl.curvature_rate)
      counter.append(lat_ctl.counter)

  X = range(len(mode))

  fig, axs = plt.subplots(2, 2, figsize=(12, 8))
  axs[0, 0].plot(X, ramp_type, label='ramp_type')
  axs[0, 0].plot(X, precision, label='precision')
  axs[0, 0].plot(X, mode, label='mode')
  axs[0, 0].plot(X, hands_off, label='hands_off')
  axs[0, 0].legend()
  axs[0, 0].set_title('ramp_type, precision, mode, hands_off')

  axs[0, 1].plot(X, checksum, label='checksum')
  axs[0, 1].plot(X, alt_checksum, label='alt_checksum')
  axs[0, 1].legend()
  axs[0, 1].set_title('checksum')

  axs[1, 0].plot(X, curvature, label='curvature')
  axs[1, 0].plot(X, path_angle, label='path_angle')
  axs[1, 0].plot(X, path_offset, label='path_offset')
  axs[1, 0].plot(X, curvature_rate, label='curvature_rate')
  axs[1, 0].legend()
  axs[1, 0].set_title('curvature, path_angle, path_offset, curvature_rate')

  axs[1, 1].plot(X, counter, label='counter')
  axs[1, 1].legend()
  axs[1, 1].set_title('counter')

  plt.show()


  # try different checksum offsets
  for offset in range(0, 0xFF):
    error_sum = 0
