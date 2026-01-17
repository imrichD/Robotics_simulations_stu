import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# ---------------------------------------------
# Maticové a transformačné funkcie (zoznamy)
# ---------------------------------------------

def Nasobenie_matic(A, B):
    # Maticové násobenie (A a B sú zoznamy zoznamov)
    m, n, p = len(A), len(B), len(B[0])
    C = [[0 for _ in range(p)] for _ in range(m)]
    for i in range(m):
        for j in range(p):
            for k in range(n):
                C[i][j] += A[i][k] * B[k][j]
    return C

def eye4():
    # 4x4 jednotková matica
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

def trans_z(d):
    # 4x4 posun v osi Z o d
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ]

def trans_x(dx):
    # 4x4 posun v osi X o dx
    return [
        [1, 0, 0, dx],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

def rot_z_deg(fi):
    # Rotácia okolo osi Z o uhol fi (v stupňoch)
    sin_val = math.sin(math.radians(fi))
    cos_val = math.cos(math.radians(fi))
    return [
        [sin_val, -cos_val, 0, 0],
        [cos_val,  sin_val, 0, 0],
        [0,        0,       1, 0],
        [0,        0,       0, 1]
    ]

def rot_y_deg(fi):
    # Rotácia okolo osi Y o uhol fi (v stupňoch)
    sin_val = math.sin(math.radians(fi))
    cos_val = math.cos(math.radians(fi))
    return [
        [cos_val,  0, sin_val, 0],
        [0,        1, 0,       0],
        [-sin_val, 0, cos_val, 0],
        [0,        0, 0,       1]
    ]

def get_xyz(T):
    # Získanie súradníc (x, y, z) z transformačnej matice T
    if len(T[0]) == 1:
        return (T[0][0], T[1][0], T[2][0])
    else:
        return (T[0][3], T[1][3], T[2][3])

# ---------------------------------------------
# Vizualizačné funkcie
# ---------------------------------------------

def plot_frame(ax, T, label="", length=0.1, arrow_length_ratio=0.1):
    # Vykreslí súradnicový systém T v 3D (x = červená, y = zelená, z = modrá)
    origin = [T[i][3] for i in range(3)]
    x_axis = [T[i][0] for i in range(3)]
    y_axis = [T[i][1] for i in range(3)]
    z_axis = [T[i][2] for i in range(3)]
    ax.quiver(origin[0], origin[1], origin[2],
              x_axis[0]*length, x_axis[1]*length, x_axis[2]*length,
              color='r', arrow_length_ratio=arrow_length_ratio)
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0]*length, y_axis[1]*length, y_axis[2]*length,
              color='g', arrow_length_ratio=arrow_length_ratio)
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0]*length, z_axis[1]*length, z_axis[2]*length,
              color='b', arrow_length_ratio=arrow_length_ratio)
    if label:
        ax.text(origin[0], origin[1], origin[2], f" {label}", fontsize=10, color='k')

def set_aspect_equal_3d(ax):
    # Nastaví rovnakú mierku pre všetky osy 3D grafu
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    ranges = limits[:, 1] - limits[:, 0]
    max_range = ranges.max()
    centers = limits.mean(axis=1)
    ax.set_xlim3d([centers[0] - max_range/2, centers[0] + max_range/2])
    ax.set_ylim3d([centers[1] - max_range/2, centers[1] + max_range/2])
    ax.set_zlim3d([centers[2] - max_range/2, centers[2] + max_range/2])

# ---------------------------------------------
# Výpočet finálnej pozície plošiny
# ---------------------------------------------

def compute_platform_position(phi1_deg, phi2_deg, phi3_deg, L1=1.0, L2=40.0, L3=1.0):
    # Spojenie rotácií a posunov na výpočet pozície koncovej plošiny
    Rz = rot_z_deg(phi1_deg)
    Ry2 = rot_y_deg(phi2_deg)
    Ry3 = rot_y_deg(phi3_deg)
    
    Tz1 = trans_z(L1)
    Tz2 = trans_z(L2)
    
    A = Nasobenie_matic(Rz, Tz1)
    B = Nasobenie_matic(Ry2, Tz2)
    
    vector_L3 = [[0], [0], [L3], [1]]
    C = Nasobenie_matic(Ry3, vector_L3)
    
    AB = Nasobenie_matic(A, B)
    Pc = Nasobenie_matic(AB, C)
    return get_xyz(Pc)

# ---------------------------------------------
# Konvexný obal, plocha a triangulácia
# ---------------------------------------------

def convex_hull_2d(points):
    # Vytvorí konvexný obal z 2D bodov
    points = sorted(points, key=lambda p: (p[0], p[1]))
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def cross(o, a, b):
    # Pomocná funkcia na výpočet 2D determinantov (orientácia)
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

def polygon_area(points):
    # Výpočet obsahu polygónu pomocou Shoelace formula
    area = 0
    n = len(points)
    for i in range(n):
        j = (i+1) % n
        area += points[i][0]*points[j][1] - points[j][0]*points[i][1]
    return abs(area) / 2

def triangulate_polygon(hull):
    # Triangulácia konvexného polygónu (fan triangulation)
    n = len(hull)
    triangles = []
    for i in range(1, n-1):
        triangles.append([0, i, i+1])
    return triangles

# ---------------------------------------------
# Výpočet vnútornej pracovnej oblasti (pri minimálnom L2)
# ---------------------------------------------

def compute_inner_workspace(phi1_vals, phi2_vals, phi3_vals, L1, L3, L2_min):
    inner_ws_points = []
    for phi1 in phi1_vals:
        for phi2 in phi2_vals:
            for phi3 in phi3_vals:
                point = compute_platform_position(phi1, phi2, phi3, L1, L2_min, L3)
                inner_ws_points.append(point)
    return np.array(inner_ws_points)

# ---------------------------------------------
# Hlavná funkcia: výpočet, obálky, vizualizácia
# ---------------------------------------------

def main():
    # Parametre mechanizmu
    L1 = 1
    phi1_deg = 0
    L2 = 40
    phi2_deg = -45
    L3 = 1
    phi3_deg = -90

    # Výpočet transformačných matíc pre jednotlivé články
    T0 = eye4()
    T0_1 = trans_z(L1)
    T1_2 = rot_z_deg(phi1_deg)
    T2_3 = Nasobenie_matic(rot_y_deg(phi2_deg), trans_x(L2))
    T3_4 = Nasobenie_matic(rot_y_deg(phi3_deg), trans_x(L3))
    
    T0_1_cum = T0_1
    T0_2_cum = Nasobenie_matic(T0_1_cum, T1_2)
    T0_3_cum = Nasobenie_matic(T0_2_cum, T2_3)
    T0_4_cum = Nasobenie_matic(T0_3_cum, T3_4)
    
    P0 = [0, 0, 0]
    p1 = get_xyz(T0_1_cum)
    p2 = get_xyz(T0_2_cum)
    p3 = get_xyz(T0_3_cum)
    p4 = get_xyz(T0_4_cum)
    
    # Výpočet množiny všetkých pracovných bodov
    phi1_vals = np.linspace(0, 360, 25)
    phi2_vals = np.linspace(0, 90, 10)
    phi3_vals = np.linspace(-180, 0, 15)
    L2_vals = np.linspace(10, 40, 4)
    
    ws_points = []
    for phi1 in phi1_vals:
        for phi2 in phi2_vals:
            for phi3 in phi3_vals:
                for L2_val in L2_vals:
                    point = compute_platform_position(phi1, phi2, phi3, L1, L2_val, L3)
                    ws_points.append(point)
    ws_points = np.array(ws_points)
    
    # Výpočet konvexnej obálky v XY rovine
    proj_xy = ws_points[:, :2]
    hull_xy = convex_hull_2d(proj_xy.tolist())
    hull_xy = np.array(hull_xy)
    z_const = np.mean(ws_points[:, 1])
    triangles_xy = triangulate_polygon(hull_xy.tolist())
    triangles_xy = np.array(triangles_xy)
    
    # Výpočet konvexnej obálky v XZ rovine
    proj_xz = ws_points[:, [0, 2]]
    hull_xz = convex_hull_2d(proj_xz.tolist())
    hull_xz = np.array(hull_xz)
    y_const = np.mean(ws_points[:, 1])
    triangles_xz = triangulate_polygon(hull_xz.tolist())
    triangles_xz = np.array(triangles_xz)
    
    # Vnútorná obálka v XZ (pri minimálnom L2)
    L2_min = np.min(L2_vals)
    inner_ws_points = compute_inner_workspace(phi1_vals, phi2_vals, phi3_vals, L1, L3, L2_min)
    proj_xz_inner = inner_ws_points[:, [0, 2]]
    hull_xz_inner = convex_hull_2d(proj_xz_inner.tolist())
    hull_xz_inner = np.array(hull_xz_inner)
    triangles_xz_inner = triangulate_polygon(hull_xz_inner.tolist())
    triangles_xz_inner = np.array(triangles_xz_inner)
    
    # Vizualizácia výsledkov
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot([P0[0], p1[0], p2[0], p3[0], p4[0]],
            [P0[1], p1[1], p2[1], p3[1], p4[1]],
            [P0[2], p1[2], p2[2], p3[2], p4[2]],
            c='orange', linewidth=3, marker='o',
            markerfacecolor='k', markeredgecolor='k')
    
    plot_frame(ax, T0, label="Base", length=5, arrow_length_ratio=0.4)
    plot_frame(ax, T0_4_cum, label="Platform", length=5, arrow_length_ratio=0.4)
    
    ax.plot_trisurf(hull_xy[:, 0], hull_xy[:, 1],
                    np.full_like(hull_xy[:, 0], z_const),
                    triangles=triangles_xy, color='cyan', alpha=0.3, shade=False)
    
    ax.plot_trisurf(hull_xz[:, 0], np.full_like(hull_xz[:, 0], y_const), hull_xz[:, 1],
                    triangles=triangles_xz, color='blue', alpha=0.3, shade=False)
    
    ax.plot_trisurf(hull_xz_inner[:, 0], np.full_like(hull_xz_inner[:, 0], y_const),
                    hull_xz_inner[:, 1],
                    triangles=triangles_xz_inner, color='red', alpha=1.0, shade=False)
    
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Pracovný priestor (XY a XZ) požiarnej výškovej plošiny")
    
    ax.set_xlim(np.min(ws_points[:, 0]) - 5, np.max(ws_points[:, 0]) + 5)
    ax.set_ylim(np.min(ws_points[:, 1]) - 5, np.max(ws_points[:, 1]) + 5)
    ax.set_zlim(np.min(ws_points[:, 2]) - 5, np.max(ws_points[:, 2]) + 5)
    
    set_aspect_equal_3d(ax)
    ax.view_init(elev=10, azim=-30)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
