# import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

# # DEFINE GLOBAL PARAMETERS
# L = 0.4
# MU = 0.3
# EP = 0.5
# dt = 0.01
# m = 0.3
# g = np.array([0., -9.81, 0.])
# rg_squared = 1./12. * (2 * L * L)  # Renamed to rg_squared
# M = np.array([[m, 0, 0], [0, m, 0], [0, 0, m * rg_squared]])
# Mi = np.array([[1./m, 0, 0], [0, 1./m, 0], [0, 0, 1./(m * rg_squared)]])
# DELTA = 0.001
# T = 150

# def get_contacts(q):
#     """
#         Return jacobian of the lowest corner of the square and distance to contact
#         :param q: <np.array> current configuration of the object
#         :return: <np.array>, <float> jacobian and distance
#     """
#     half = L / 2 
#     corner = np.array([[half, half], [-half, half], [-half, -half], [half, -half]])
#     rot_mat = np.array([[np.cos(q[-1]), -np.sin(q[-1])], [np.sin(q[-1]), np.cos(q[-1])]])
#     rot_corner = q[0:2] + np.dot(corner, rot_mat.T)
#     idx = np.argmin(rot_corner[:, 1])
#     lower = rot_corner[idx]
#     phi = lower[1]
#     r = lower - q[0:2]
#     J_t = np.array([1, 0, -r[1]])
#     J_n = np.array([0, 1, r[0]])  
#     jac = np.column_stack((J_t, J_n))
#     return jac, phi

# def form_lcp(jac, v):
#     """
#         Return LCP matrix and vector for the contact
#         :param jac: <np.array> jacobian of the contact point
#         :param v: <np.array> velocity of the center of mass
#         :return: <np.array>, <np.array> V and p
#     """
#     Jt = jac[:,0]
#     Jn = jac[:,1]
#     fe = m * g
#     V = np.zeros((4, 4))
#     V[0] = [Jn.T @ np.linalg.inv(M) @ Jn * dt, -Jn.T @ np.linalg.inv(M) @ Jt * dt, Jn.T @ np.linalg.inv(M) @ Jt * dt, 0]
#     V[1] = [-Jt.T @ np.linalg.inv(M) @ Jn * dt, Jt.T @ np.linalg.inv(M) @ Jt * dt, -Jt.T @ np.linalg.inv(M) @ Jt * dt, 1]
#     V[2] = [Jt.T @ np.linalg.inv(M) @ Jn * dt, -Jt.T @ np.linalg.inv(M) @ Jt * dt, Jt.T @ np.linalg.inv(M) @ Jt * dt, 1]
#     V[3] = [MU, -1, -1, 0]
#     p = np.zeros((4,))
#     p[0] = Jn.T @ ((1 + EP) * v  + dt * np.linalg.inv(M) @ fe)
#     p[1] = -Jt.T @ (v  + dt * np.linalg.inv(M) @ fe)
#     p[2] = Jt.T @ (v  + dt * np.linalg.inv(M) @ fe)
#     return V, p

# def step(q, v):
#     """
#         predict next config and velocity given the current values
#         :param q: <np.array> current configuration of the object
#         :param v: <np.array> current velocity of the object
#         :return: <np.array>, <np.array> q_next and v_next
#     """
#     jac, phi = get_contacts(q)
#     Jt = jac[:,0]
#     Jn = jac[:,1]
#     fe = m * g
#     qp = np.array([0, DELTA, 0])
#     v_next = None
#     q_next = None
#     if phi < DELTA:
#         V, p = form_lcp(jac, v)
#         fc = lcp_solve(V, p)
#         v_next = v + dt * np.linalg.inv(M) @ (fe + Jn * fc[0] - Jt * fc[1] + Jt * fc[2])
#         q_next = q + dt * v_next + qp
#     else:
#         v_next = v + dt * np.linalg.inv(M) @ fe
#         q_next = q + dt * v_next
#     return q_next, v_next

# def simulate(q0, v0):
#     """
#         predict next config and velocity given the current values
#         :param q0: <np.array> initial configuration of the object
#         :param v0: <np.array> initial velocity of the object
#         :return: <np.array>, <np.array> q and v trajectory of the object
#     """
#     q = np.zeros((3, T))
#     v = np.zeros((3, T))
#     q[:, 0] = q0
#     v[:, 0] = v0
#     for t in range(T - 1):
#         q[:, t+1], v[:, t+1] = step(q[:, t], v[:, t])
#     return q, v

# def lcp_solve(V, p, pivtol=1e-8):
#     """
#         Solves the Linear Complementary Problem Equations
#         :param V: <np.array> matrix of the LCP
#         :param p: <np.array> vector of the LCP
#         :return: LCP solution
#     """
#     rayTerm = False
#     loopcount = 0
#     if (p >= 0.).all():
#         w = p
#         z = np.zeros_like(p)
#         retcode = 0.
#     else:
#         dimen = V.shape[0]
#         tableau = np.hstack([np.eye(dimen), -V, -np.ones((dimen, 1)), np.asarray(np.asmatrix(p).T)])
#         basis = list(range(dimen))
#         locat = np.argmin(tableau[:, 2 * dimen + 1])
#         basis[locat] = 2 * dimen
#         cand = locat + dimen
#         pivot = tableau[locat, :] / tableau[locat, 2 * dimen]
#         tableau -= tableau[:, 2 * dimen:2 * dimen + 1] * pivot
#         tableau[locat, :] = pivot
#         oldDivideErr = np.seterr(divide='ignore')['divide']
#         while np.amax(basis) == 2 * dimen:
#             loopcount += 1
#             eMs = tableau[:, cand]
#             missmask = eMs <= 0.
#             quots = tableau[:, 2 * dimen + 1] / eMs
#             quots[missmask] = np.Inf
#             locat = np.argmin(quots)
#             if abs(eMs[locat]) > pivtol and not missmask.all():
#                 pivot = tableau[locat, :] / tableau[locat, cand]
#                 tableau -= tableau[:, cand:cand + 1] * pivot
#                 tableau[locat, :] = pivot
#                 oldVar = basis[locat]
#                 basis[locat] = cand
#                 if oldVar >= dimen:
#                     cand = oldVar - dimen
#                 else:
#                     cand = oldVar + dimen
#             else:
#                 rayTerm = True
#                 break
#         np.seterr(divide=oldDivideErr)
#         vars = np.zeros(2 * dimen + 1)
#         vars[basis] = tableau[:, 2 * dimen + 1]
#         w = vars[:dimen]
#         z = vars[dimen:2 * dimen]
#         retcode = vars[2 * dimen]
#     if rayTerm:
#         retcode = (2, retcode, loopcount)
#     else:
#         retcode = (1, retcode, loopcount)
#     return z

# def render(traj):
#     """
#         Renders the trajectory using matplotlib
#         :param traj: <np.array> configuration trajectory
#         :return: None
#     """
#     fig = plt.figure(figsize=(5, 5))
#     ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-0.1, 2))
#     ax.set_aspect('equal')
#     ax.grid()

#     line, = ax.plot([], [], 'o-', lw=2)
#     time_template = 'time = %.1fs'
#     time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
#     dt = 0.01

#     def animate(i):
#         local_corners = np.array([[-0.2, -0.2, 0.2, 0.2],
#                                   [-0.2, 0.2, 0.2, -0.2],
#                                   [1, 1, 1, 1]])
#         H = np.array([[np.cos(traj[2, i]), -np.sin(traj[2, i]), traj[0, i]],
#                       [np.sin(traj[2, i]), np.cos(traj[2, i]), traj[1, i]],
#                       [0., 0., 1]])
#         world_corners = H @ local_corners

#         line.set_data(world_corners[0, :], world_corners[1, :])
#         time_text.set_text(time_template % (i * dt))
#         return line, time_text

#     ani = animation.FuncAnimation(
#         fig, animate, traj.shape[1], interval=dt * 3000, blit=True)
#     plt.show()

# if __name__ == "__main__":
#     q0 = np.array([0.0, 1.5, np.pi / 180. * 30.])
#     v0 = np.array([0., -0.2, 0.])
#     q, v = simulate(q0, v0)

#     plt.plot(q[1, :])
#     plt.show()

#     render(q)
import PolygonCollision

#Create Squares
polygon1 = PolygonCollision.shape.Shape(vertices = [(0, 0), (0, 10), (10, 10), (10, 0)])
polygon2 = PolygonCollision.shape.Shape(vertices = [(5, 5), (5, 15), (15, 15), (15, 5)], fill=False)

#Create Circle
circle = PolygonCollision.shape.Shape(vertices = [(30, 30)], radius = 5)

#Output The Size And Posiotion Of The Shapes
print('polygon1:', polygon1.get_width(), polygon1.get_height())

#Check For Polygons Collision
if polygon1.collide(polygon2):
    print('POLYGON COLLISION!!!')
else:
    print('no polygon colllision')

#Check For Polygons + Circle Collision
if polygon1.collide(polygon2):
    print('CIRCLE COLLISION!!!')
else:
    print('no circle colllision')