#!/usr/bin/env python

from math import sqrt

Inf = float('inf')

def calc_planar_dist(p,q):
    return sqrt( (p[0]-q[0])**2 + (p[1]-q[1])**2 )

# https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
def get_planar_orientation_3_points(p,q,r):
    val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
    if (val > 0):
        # Clockwise orientation
        return 1
    elif (val < 0):
        # Counterclockwise orientation
        return 2
    else:
        # Collinear orientation
        return 0

def point_planar_lies_segment(q, S):
    p = S[0]; r = S[1]
    if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
        return True
    return False

def do_2_planar_segments_insersect(S1,S2):
    p1 = S1[0]; q1 = S1[1]
    p2 = S2[0]; q2 = S2[1]
    o1 = get_planar_orientation_3_points(p1, q1, p2)
    o2 = get_planar_orientation_3_points(p1, q1, q2)
    o3 = get_planar_orientation_3_points(p2, q2, p1)
    o4 = get_planar_orientation_3_points(p2, q2, q1)
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
    # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
    if ((o1 == 0) and point_planar_lies_segment(p2 ,(p1,q1))):
        return True
    # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
    if ((o2 == 0) and point_planar_lies_segment(q2, (p1,q1))):
        return True
    # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
    if ((o3 == 0) and point_planar_lies_segment(p1, (p2,q2))):
        return True
    # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
    if ((o4 == 0) and point_planar_lies_segment(q1, (p2,q2))):
        return True
    # If none of the cases
    return False