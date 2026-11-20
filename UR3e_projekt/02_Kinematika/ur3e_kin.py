"""Kinematika UR3e: direktna (DH) i numericka inverzna kinematika."""
import numpy as np

# Standardni DH parametri (UR support article), metri i radijani
ROBOTS = {
    "UR3e": dict(a=[0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0],
                 d=[0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921]),
    "UR3":  dict(a=[0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0],
                 d=[0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819]),
}
ALPHA = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
A = ROBOTS["UR3e"]["a"]
D = ROBOTS["UR3e"]["d"]


def use_robot(name):
    """Odabir DH parametara: 'UR3e' (zadatak) ili 'UR3' (ugradjeni model)."""
    global A, D
    A = ROBOTS[name]["a"]
    D = ROBOTS[name]["d"]


def base_to_dh(T):
    """UR 'Base' sustav (x naprijed) u DH sustav baze (zarotiran za 180 deg oko z)."""
    Rz = np.diag([-1.0, -1.0, 1.0, 1.0])
    return Rz @ T


def dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0.0,    sa,     ca,    d],
                     [0.0,   0.0,    0.0,  1.0]])


def fk(q, tool_z=0.0):
    """Poza TCP-a u koordinatnom sustavu baze (4x4)."""
    T = np.eye(4)
    for i in range(6):
        T = T @ dh(q[i], D[i], A[i], ALPHA[i])
    Tt = np.eye(4)
    Tt[2, 3] = tool_z
    return T @ Tt


def rot_err(R_t, R_c):
    """Vektor malene rotacijske greske (os*kut) u svjetskom sustavu."""
    Re = R_t @ R_c.T
    return 0.5 * np.array([Re[2, 1] - Re[1, 2],
                           Re[0, 2] - Re[2, 0],
                           Re[1, 0] - Re[0, 1]])


def jac(q, tool_z, eps=1e-6):
    J = np.zeros((6, 6))
    T0 = fk(q, tool_z)
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += eps
        T1 = fk(dq, tool_z)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
        J[3:, i] = rot_err(T1[:3, :3], T0[:3, :3]) / eps
    return J


def ik(T_target, q0, tool_z=0.0, iters=200, lam=0.05, tol=1e-5):
    """Damped least squares IK iz pocetne konfiguracije q0 (kontinuitet putanje)."""
    q = np.array(q0, float)
    for _ in range(iters):
        T = fk(q, tool_z)
        e = np.hstack((T_target[:3, 3] - T[:3, 3],
                       rot_err(T_target[:3, :3], T[:3, :3])))
        if np.linalg.norm(e) < tol:
            return q, True
        J = jac(q, tool_z)
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
        q = q + dq
    T = fk(q, tool_z)
    e = np.hstack((T_target[:3, 3] - T[:3, 3],
                   rot_err(T_target[:3, :3], T[:3, :3])))
    return q, bool(np.linalg.norm(e) < 1e-4)


def pose_down(x, y, z, yaw=0.0):
    """Alat gleda okomito dolje (os z alata = -z baze)."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([[cy,  sy,  0.0],
                  [sy, -cy,  0.0],
                  [0.0, 0.0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def line_path(q0, p0, p1, yaw, tool_z, speed, dt):
    """Pravocrtno gibanje TCP-a (u 'Base' sustavu) od p0 do p1.
    Vraca listu konfiguracija zglobova, jednu za svaki korak simulacije."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    n = max(2, int(np.ceil(np.linalg.norm(p1 - p0) / (speed * dt))))
    q = np.array(q0, float)
    path = []
    for k in range(1, n + 1):
        p = p0 + (p1 - p0) * k / n
        q, ok = ik(base_to_dh(pose_down(p[0], p[1], p[2], yaw)), q, tool_z)
        if not ok:
            raise ValueError(f"IK ne konvergira u tocki {np.round(p, 3)}")
        path.append(q.copy())
    return path
