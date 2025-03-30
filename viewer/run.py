from itertools import combinations
from math import atan, pi
from time import monotonic
from cscore import CameraServer
from networktables import NetworkTables
import numpy as np
import pyrealsense2 as rs
import cv2

w, h, fps = 640, 480, 30
pipe = rs.pipeline()

meter_scale = .5 # i have trust in this goated number
colorizer = rs.colorizer()
colorizer.set_option(rs.option.visual_preset, 1)
colorizer.set_option(rs.option.min_distance, 0)
colorizer.set_option(rs.option.max_distance, meter_scale)
colorizer.set_option(rs.option.color_scheme, 3) # black to white (0->255)

config = rs.config()
config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
config.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)

hff = rs.hole_filling_filter(1) # farest from around
aligner = rs.align(rs.stream.color)

profile = pipe.start(config)

NetworkTables.setNetworkIdentity("skynet")
NetworkTables.initialize("10.1.72.2")
viewer_nt = NetworkTables.getTable("Viewer")
cs_video = CameraServer.putVideo("Video", w, h)
cs_depth = CameraServer.putVideo("Depth", w//2, h//2)

dist_thresh = 30
time_thresh = .050 # 100 ms MAX (needs more tuning)
ds_offset = 5/6 # bc the color cam is slightly to the left
auto_offset = .15 # also the depth too

def find(parent, line):
    return line if parent[line] == line else find(parent, parent[line])

def union(parent, line1, line2):
    parent[find(parent, line2)] = find(parent, line1)

# [(time, line), ...]
prev_lines = []

while True:
    start_time = monotonic()

    prev_lines = list(filter(lambda pl: start_time-pl[0] < time_thresh, prev_lines))

    frames = aligner.process(pipe.wait_for_frames())
    depth = colorizer.colorize(hff.process(frames.get_depth_frame()))
    color = frames.get_color_frame()

    color_img_pre = cv2.cvtColor(np.asanyarray(color.get_data()), cv2.COLOR_RGB2BGR)
    dist_img_pre = cv2.cvtColor(np.asanyarray(depth.get_data()), cv2.COLOR_RGB2GRAY)
    # half it for performance during thinning (320x240)
    dist_img_pre = cv2.resize(dist_img_pre, (w//2, h//2))
    
    color_img = cv2.rotate(color_img_pre, cv2.ROTATE_180)
    dist_img = cv2.rotate(dist_img_pre, cv2.ROTATE_180)

    thresh_img = cv2.inRange(dist_img, 0, 240)
    thresh_img[int((h/2)*(5/6)):, :] = 0
    skel = cv2.ximgproc.thinning(thresh_img, thinningType=cv2.ximgproc.THINNING_GUOHALL)

    new_lines = cv2.HoughLinesP(skel, 3, np.pi/180, 1, None, 52, 20)
    new_lines = new_lines if new_lines is not None else []
    new_lines = list(filter(
        lambda l: abs(atan((l[2]-l[0])/(l[3]-l[1]))) < pi/12 and abs((l[2]+l[0])/2 - w/4)/(w/2) < .28,
        map(lambda l: tuple(l[0]), new_lines)))

    prev_lines = list(filter(lambda pl: start_time-pl[0] < time_thresh, prev_lines))
    lines = [*new_lines, *map(lambda pl: tuple(pl[1]), prev_lines)]
    prev_lines.extend(map(lambda l: (start_time, l), new_lines))

    # union-find algorithm time!
    parent = dict((line, line) for line in lines)

    for line1, line2 in combinations(lines, 2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        midx1, midy1 = ((x1+x2)/2, (y1+y2)/2)
        midx2, midy2 = ((x3+x4)/2, (y3+y4)/2)
        if abs(midx2-midx1) < dist_thresh:
            union(parent, line2, line1)

    groups = {}
    for line in lines:
        rep = find(parent, line)
        if rep not in groups.keys():
            groups[rep] = list()
        groups[rep].append(line)

    avg_lines = []
    for group in groups.values():
        avg = np.uint(np.mean(group, axis=0))
        avg_lines.append(avg)

    offsets = list(map(lambda l: abs((l[2]+l[0])/2 - w/4 + auto_offset), avg_lines))

    dbg_img = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)
    dbg_img = cv2.bitwise_or(dbg_img, cv2.merge([np.zeros_like(skel), skel, skel]))

    for x1, y1, x2, y2 in lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 0, 255), 1)
    for x1, y1, x2, y2 in avg_lines:
        dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    in_dist = False
    x_dist = float("nan")
    if avg_lines:
        x1, y1, x2, y2 = avg_lines[np.argmin(offsets)]
        midpx = (x1+x2)/2
        midpy = (y1+y2)/2
        x_dist = (midpx - w//4)/(w//2) + auto_offset
        in_dist = abs(x_dist) < 0.025

        # dbg_img = cv2.line(dbg_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        dbg_img = cv2.line(dbg_img, (int(midpx), int(midpy)), ((w//4) - int(auto_offset * w//4), int(midpy)), (0, 255, 255))
        dbg_img = cv2.drawMarker(dbg_img, (int(midpx), int(midpy)), (0, 255, 255), cv2.MARKER_CROSS, 10, 2)
        dbg_img = cv2.putText(dbg_img, f"{x_dist=:.2f}", (20, 40), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1)
    
    end_time = monotonic()

    viewer_nt.putNumber("CandidateMetersX", x_dist)
    viewer_nt.putNumberArray("Posts", offsets)

    dbg_img = cv2.putText(
        dbg_img, f"fps: {1/(end_time-start_time):.0f}",
        (20, 20), cv2.FONT_HERSHEY_COMPLEX, .5, (0, 0, 255), 1)
    color_img = cv2.line(
        color_img,
        (int(w//2*ds_offset), 0),
        (int(w//2*ds_offset), h),
        (0, int(in_dist*255), int((not in_dist)*255)), 10)

    cs_video.putFrame(color_img)
    cs_depth.putFrame(dbg_img)