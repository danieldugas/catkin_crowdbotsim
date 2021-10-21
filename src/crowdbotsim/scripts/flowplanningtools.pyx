# distutils: language=c++

from libcpp cimport bool
from libcpp.queue cimport priority_queue as cpp_priority_queue
from libcpp.pair cimport pair as cpp_pair
import numpy as np
cimport numpy as np
from cython.operator cimport dereference as deref
cimport cython
from math import sqrt
from libc.math cimport cos as ccos
from libc.math cimport sin as csin
from libc.math cimport acos as cacos
from libc.math cimport sqrt as csqrt
from libc.math cimport floor as cfloor
from libc.math cimport exp as cexp
from libc.math cimport M_PI as cpi

def fast_discrete_model(xx, yy, obs_odom, sdf, gamma, sigma):
    vxgrid = np.zeros_like(xx, dtype=np.float32)
    vygrid = np.zeros_like(xx, dtype=np.float32)
    rhogrid = np.zeros_like(xx, dtype=np.float32)
    cfast_discrete_model(xx, yy, obs_odom, sdf, gamma, sigma, vxgrid, vygrid, rhogrid)
    return vxgrid, vygrid, rhogrid

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef cfast_discrete_model(
    np.float32_t[:, ::1] xx,
    np.float32_t[:, ::1] yy,
    np.float32_t[:, ::1] obs_odom,
    np.float32_t[:, ::1] sdf,
    np.float32_t gamma,
    np.float32_t sigma,
    # outputs
    np.float32_t[:, ::1] vxgrid,
    np.float32_t[:, ::1] vygrid,
    np.float32_t[:, ::1] rhogrid,
):
    cdef int i
    cdef int j
    cdef int n
    cdef int i_cells = xx.shape[0]
    cdef int j_cells = xx.shape[1]
    cdef int n_observations = len(obs_odom)
    cdef np.float32_t x
    cdef np.float32_t y
    cdef np.float32_t dr2
    cdef np.float32_t mux
    cdef np.float32_t muy
    cdef np.float32_t vxmeas
    cdef np.float32_t vymeas
    cdef np.float32_t alphax
    cdef np.float32_t alphay
    cdef np.float32_t gaussian
    cdef np.float32_t weightedx_sum
    cdef np.float32_t weightedy_sum
    cdef np.float32_t alphax_sum
    cdef np.float32_t alphay_sum
    cdef np.float32_t rho_sum
    for i in range(i_cells):
        for j in range(j_cells):
            # v_star = (alphax1 * vx1 + alphax2 * vx2 + ...) / (alphax1 + alphax2 + ...)
            # rho_star = (rho1 + rho2 + ...)
            weightedx_sum = 0
            weightedy_sum = 0
            alphax_sum = 0
            alphay_sum = 0
            rho_sum = 0
            x = xx[i, j]
            y = yy[i, j]
            for n in range(n_observations):
                # currently we care about (dvx/dy)^2 and (dvy/dx)^2,
                # not (dvx/dx)^2 and (dvy/dy)^2
                # for simplicity, we replace dx and dy with dr (absolute distance)
                dr2 = ((obs_odom[n, 0] - x)**2 + (obs_odom[n, 1] - y)**2)
                vxmeas = obs_odom[n, 3]
                vymeas = obs_odom[n, 4]
                alphax = gamma * cexp(-dr2)
                alphay = gamma * cexp(-dr2)
                weightedx_sum += alphax * vxmeas
                weightedy_sum += alphay * vymeas
                alphax_sum += alphax
                alphay_sum += alphay
                # gaussian model for density (smoothing filter preserving p/m2)
                # a bivariate normal distribution smoothes the discrete signal, but has the same area
                mux = obs_odom[n, 0]
                muy = obs_odom[n, 1]
                gaussian = 1. / (2. * cpi * sigma*sigma) * cexp(
                    -1./2. * (((x - mux) / sigma)**2 + ((y - muy) / sigma)**2)
                )
                rho_sum += gaussian
            # treat closest obstacle as observation
            dr2 = sdf[i, j]**2
            alphax = gamma * cexp(-dr2)
            alphay = gamma * cexp(-dr2)
            alphax_sum += alphax
            alphay_sum += alphay
            if alphax_sum == 0:
                alphax_sum = 0.000001
            if alphay_sum == 0:
                alphay_sum = 0.000001
            # fill values
            vxgrid[i, j] = weightedx_sum / alphax_sum
            vygrid[i, j] = weightedy_sum / alphay_sum
            rhogrid[i, j] = rho_sum

def crowdflow_dijkstra(goal_boundary_ij,
                       crowd_vel_x, crowd_vel_y, crowd_density, crowd_turbulence,
                       mask, max_robot_vel, mu, static_nudge_vel,
                       inv_value=None):
    """ Dijkstra algorithm, optimized for the case where crowd velocity is given at every point, 
    and leads to directed edge costs which depend on position, crowd density, velocity, direction.

    Note: the algorithm expands from the goal, meaning that edge costs are reversed

    crowd density in p/m2
    mu is the empirical friction coefficient
        (good value is 10 - leads to max against-crowd velocity of 0.1 m/s for rho of 1 p/m2)
    velocities in m/s
    mask (uint8): 1 where static obstacles

    returns
    ---
    result: minimum cost to goal at each point
    optimal_vel: (x, y) velocity along minimum-cost path to goal at each point
    """
    # initialize field to large value
    if inv_value is None:
        inv_value = np.inf
    result = np.ones_like(crowd_vel_x, dtype=np.float32) * inv_value
    optimal_vel_x = np.zeros_like(crowd_vel_x, dtype=np.float32)
    optimal_vel_y = np.zeros_like(crowd_vel_x, dtype=np.float32)
    shape0, shape1 = result.shape
    ccrowdflow_dijkstra(shape0, shape1, goal_boundary_ij,
                        result, optimal_vel_x, optimal_vel_y,
                        crowd_vel_x, crowd_vel_y, crowd_density, crowd_turbulence,
                        mask, max_robot_vel, mu, static_nudge_vel, inv_value)
    return result, optimal_vel_x, optimal_vel_y

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef ccrowdflow_dijkstra(int shape0, int shape1, np.int64_t[:, ::1] goal_boundary_ij,
                         np.float32_t[:, ::1] tentative,
                         np.float32_t[:, ::1] optimal_vel_x,
                         np.float32_t[:, ::1] optimal_vel_y,
                         np.float32_t[:, ::1] crowd_vel_x,
                         np.float32_t[:, ::1] crowd_vel_y,
                         np.float32_t[:, ::1] crowd_density,
                         np.float32_t[:, ::1] crowd_turbulence,
                         np.uint8_t[:, ::1] mask,
                         np.float32_t max_robot_vel,
                         np.float32_t mu,
                         np.float32_t static_nudge_vel,
                         np.float32_t inv_value):
    # Initialize bool arrays
    cdef np.uint8_t[:, ::1] open_ = np.ones((shape0, shape1), dtype=np.uint8)
    # Mask (close) unattainable nodes
    for i in range(shape0):
        for j in range(shape1):
            if mask[i, j]:
                open_[i, j] = 0
    # Start at the goal location
    cdef cpp_priority_queue[cpp_pair[np.float32_t, cpp_pair[np.int64_t, np.int64_t]]] priority_queue
    for goal_ij in goal_boundary_ij:
        tentative[goal_ij[0], goal_ij[1]] = 0
        priority_queue.push(
            cpp_pair[np.float32_t, cpp_pair[np.int64_t, np.int64_t]](0, cpp_pair[np.int64_t, np.int64_t](goal_ij[0], goal_ij[1]))
        )
    cdef cpp_pair[np.float32_t, cpp_pair[np.int64_t, np.int64_t]] popped
    cdef np.int64_t popped_idxi
    cdef np.int64_t popped_idxj
    cdef np.int64_t[:, ::1] neighbor_offsets
    neighbor_offsets = np.array([
        [0, 1], [1, 0], [ 0,-1], [-1, 0], # first row must be up right down left
        [1, 1], [1,-1], [-1, 1], [-1,-1]], dtype=np.int64)
    cdef np.int64_t n_neighbor_offsets = len(neighbor_offsets)
    cdef np.int64_t len_i = tentative.shape[0]
    cdef np.int64_t len_j = tentative.shape[1]
    cdef np.int64_t smallest_tentative_id
    cdef np.float32_t value
    cdef np.float32_t smallest_tentative_value
    cdef np.int64_t node_idxi
    cdef np.int64_t node_idxj
    cdef np.int64_t neighbor_idxi
    cdef np.int64_t neighbor_idxj
    cdef np.int64_t oi
    cdef np.int64_t oj
    cdef np.int64_t currenti
    cdef np.int64_t currentj
    cdef np.float32_t edge_length
    cdef np.float32_t edge_x
    cdef np.float32_t edge_y
    cdef np.float32_t vcx
    cdef np.float32_t vcy
    cdef np.float32_t rho
    cdef np.float32_t vc_norm2
    cdef np.float32_t vc_norm
    cdef np.float32_t lbda_0
    cdef np.float32_t l_nudge
    cdef np.float32_t l_star
    cdef np.float32_t l_max
    cdef np.float32_t l_opt
    cdef np.float32_t delta_min2
    cdef np.float32_t delta_max
    cdef np.float32_t time_cost
    cdef np.float32_t edge_cost
    cdef np.float32_t new_cost
    cdef np.float32_t old_cost
    cdef np.uint8_t[::1] blocked = np.zeros((8), dtype=np.uint8)
    while not priority_queue.empty():
        currenti = -1
        currentj = -1
        # Pop the node with the smallest tentative value from the to_visit list
        while not priority_queue.empty():
            popped = priority_queue.top()
            priority_queue.pop()
            popped_idxi = popped.second.first
            popped_idxj = popped.second.second
            # skip nodes which are already closed (stagnant duplicates in the heap)
            if open_[popped_idxi, popped_idxj] == 1:
                currenti = popped_idxi
                currentj = popped_idxj
                break
        if currenti == -1: # didn't find an open node in the priority queue
            break
        # Iterate over neighbors
        for n in range(n_neighbor_offsets):
            # Indices for the neighbours
            oi = neighbor_offsets[n, 0]
            oj = neighbor_offsets[n, 1]
            neighbor_idxi = currenti + oi
            neighbor_idxj = currentj + oj
            # exclude forbidden/explored areas of the grid
            if neighbor_idxi < 0:
                continue
            if neighbor_idxi >= len_i:
                continue
            if neighbor_idxj < 0:
                continue
            if neighbor_idxj >= len_j:
                continue
            # check whether path is obstructed (for 16/32 connectedness)
            if n < 4:
                blocked[n] = mask[neighbor_idxi, neighbor_idxj]
            elif n < 8:
                blocked[n] = mask[neighbor_idxi, neighbor_idxj]
            # Exclude obstructed jumps (for 16/32 connectedness)
            if n > 4: # for example, prevent ur if u is blocked
                # assumes first row of offsets is up right down left (see offset init!)
                if (oj > 0 and blocked[0]) or \
                   (oi > 0 and blocked[1]) or \
                   (oj < 0 and blocked[2]) or \
                   (oi < 0 and blocked[3]):
                    continue
            # Exclude invalid neighbors
            if not open_[neighbor_idxi, neighbor_idxj]:
                continue
            # Get cost from neighbor to current node
            # first, we need to find optimal velocity given crowd velocity
            # ------------- this could be a function
            # unit vector in edge direction, from neighbor to current
            edge_length = csqrt(oi**2 + oj**2)
            edge_x = - oi / edge_length
            edge_y = - oj / edge_length
            # crowd velocity at neighbor point
            vcx = crowd_vel_x[neighbor_idxi, neighbor_idxj]
            vcy = crowd_vel_y[neighbor_idxi, neighbor_idxj]
            vc_norm2 = vcx*vcx + vcy*vcy
            # delta max according to friction model (maximum deviation from crowd vel)
            # + turbulence (difference between norm of average vel and average norm of vel)
            turbulence = crowd_turbulence[neighbor_idxi, neighbor_idxj]
            rho = crowd_density[neighbor_idxi, neighbor_idxj]
            delta_max = 1. / (rho * mu) + turbulence
            # velocity along edge which minimizes friction
            l_star = edge_x*vcx + edge_y*vcy # dot product
            delta_min2 = vc_norm2 - l_star*l_star
            # maximum velocity in nudge regime
            vc_norm = csqrt(vc_norm2)
            lbda_0 = (static_nudge_vel + vc_norm) * mu * rho
            l_nudge = min(static_nudge_vel / lbda_0, static_nudge_vel)
            # maximum velocity within friction constraint on current edge
            l_max = -1.
            if delta_min2 <= delta_max*delta_max: # if there is intersection between friction circle and line
                l_max = l_star + csqrt(delta_max*delta_max - delta_min2)
            if l_max <= 0: # not allowed to move according to hard constraint
                # maybe the solution is missed because vc lies between discrete directions?
                # closest if theta <= 22.5deg, a.k.a cos(theta) >= cos(pi/8)
                # close if theta <= 45 deg
                # cos theta = vc dot e / (|vc||e|)
                cos_theta = l_star / vc_norm
                if cos_theta >= ccos(cpi/4.):
                    l_max = l_star
            # find optimal velocity
            # in the hard-constraint case, cost-optimal l is l_max
            # in the soft-constraint case, need to solve a cost minimization between time and friction cost
            # l* > l > l_max
            # if alpha == np.inf 
            # l = l_star # (except if l_star is negative)
            # if alpha == 0
            # l = l_max
            l_opt = max(l_max, l_nudge)
            # apply robot velocity constraint
            l_opt = min(l_opt, max_robot_vel)
            if l_opt <= 0:
                continue
            # ---------------------------------------
            time_cost = edge_length / l_opt
            edge_cost = time_cost # + alpha * friction_cost
            new_cost = (
                tentative[currenti, currentj] + edge_cost
            )
            old_cost = tentative[neighbor_idxi, neighbor_idxj]
            if new_cost < old_cost or old_cost == inv_value:
                tentative[neighbor_idxi, neighbor_idxj] = new_cost
                optimal_vel_x[neighbor_idxi, neighbor_idxj] = l_opt * edge_x
                optimal_vel_y[neighbor_idxi, neighbor_idxj] = l_opt * edge_y
                # Add neighbor to priority queue
                priority_queue.push(
                    cpp_pair[np.float32_t, cpp_pair[np.int64_t, np.int64_t]](
                        -new_cost, cpp_pair[np.int64_t, np.int64_t](neighbor_idxi, neighbor_idxj))
                )
        # Close the current node
        open_[currenti, currentj] = 0

def calculate_crowd_velocity(crowd, previous_crowd, dt):
    _O = 6
    crowd_odom = np.zeros((len(crowd), _O), dtype=np.float32)
    ccalculate_crowd_velocity(
        len(crowd),
        np.array(crowd).astype(np.float32),
        np.array(previous_crowd).astype(np.float32),
        dt,
        crowd_odom
    )
    return crowd_odom

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef ccalculate_crowd_velocity(
    int n_persons,
    np.float32_t[:, ::1] crowd,
    np.float32_t[:, ::1] previous_crowd,
    np.float32_t dt,
    np.float32_t[:, ::1] crowd_odom,
):
    cdef int i
    cdef np.float32_t id_
    cdef np.float32_t prev_id
    cdef np.float32_t closest
    cdef np.float32_t delta_x
    cdef np.float32_t delta_y
    cdef np.float32_t distance
    cdef np.float32_t[::1] delta
    cdef np.float32_t[::1] vel
    cdef np.float32_t[::1] person
    for i in range(n_persons):
        crowd_odom[i, 0] = crowd[i, 1]
        crowd_odom[i, 1] = crowd[i, 2]
        crowd_odom[i, 2] = crowd[i, 3]
    # find closest (may be duplicates due to portals) person with matching id
    for i in range(n_persons):
        id_ = crowd[i, 0]
        closest = np.inf
        for j in range(n_persons):
            prev_id = previous_crowd[j, 0]
            if id_ == prev_id:
                delta_x = crowd[i, 1] - previous_crowd[j, 1]
                delta_y = crowd[i, 2] - previous_crowd[j, 2]
                distance2 = delta_x*delta_x + delta_y*delta_y
                if distance2 <= closest:
                    closest = distance2
                    crowd_odom[i, 3] = delta_x / dt
                    crowd_odom[i, 4] = delta_y / dt
                    crowd_odom[i, 5] = cangle_difference(crowd[i, 3], previous_crowd[j, 3]) / dt

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef cangle_difference(np.float32_t a, np.float32_t b):
    """ returns smallest angle a - b """
    cdef np.float32_t delta = a - b
    delta = (delta + cpi) % (2.*cpi) - cpi
    return delta
