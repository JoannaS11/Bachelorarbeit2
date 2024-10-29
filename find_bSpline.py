import numpy as np
import geomdl.fitting
import math
from datetime import datetime
import os
import geomdl.exchange


def get_bSpline(pcd, sample_size):
    pcd_np = np.asarray(pcd.points)

    # interpolate line and upsample
    line_pc_array = np.ndarray.tolist(pcd_np)
    b_spline = geomdl.fitting.interpolate_curve(line_pc_array, 2)
    b_spline.sample_size = sample_size * b_spline.sample_size

    return b_spline


def get_spline_length(b_spline):
    ctr_points = np.asarray(b_spline.ctrlpts)
    ctr_size = b_spline.ctrlpts_size
    appr_length_bspline = 0
    # sum up the distances between the control points
    for i in range(ctr_size - 1):
        appr_length_bspline += math.dist(ctr_points[i, :], ctr_points[i + 1, :])

    return appr_length_bspline


def export_spline_as_json(b_spline, output_name_without_json, folder):
    # get current date and time
    now = datetime.now()
    date_time = now.strftime("%d-%m-%Y_%H-%M-%S")
    # export as json
    name = f"{date_time}_{output_name_without_json}.json"
    geomdl.exchange.export_json(b_spline, os.path.join(os.getcwd(), *folder, name))

    return name
