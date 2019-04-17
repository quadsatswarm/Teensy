import math
from re import split

import numpy
import struct
from digi.xbee.devices import XBeeDevice
# from scipy.special import lambertw
import time
import warnings

# Variables for artificial position field calculations
AGENT_COUNT = 1  # Number of Agents/Quadcopters in use
NODE_COUNT = 1  # Number of Attractive Nodes
DIMENSION_COUNT = 3  # Dimensions

MAX_ATTRACTIVE_FORCE = 10  # Max Attractive Force
EQUILIBRIUM_DISTANCE = 2  #
STEEPNESS_COEFFICIENT = 100  #
SMOOTHNESS_COEFFICIENT = 5  #
ATTRACTIVE_FRACTION = (1 / (2 * math.e)) * (NODE_COUNT / AGENT_COUNT)
REPULSIVE_FRACTION = (1 / (2 * math.e)) * (AGENT_COUNT / NODE_COUNT)

ATTRACTIVE_GAIN = MAX_ATTRACTIVE_FORCE / NODE_COUNT  # Attractive Gain

# compiler gives a warning that the following is stored as a real value from a complex value
# which discards the imaginary part, this should be fine since the imaginary portion should be 0 because the
# branch value passed to the function, the warning below should fix this
warnings.filterwarnings("ignore", message="Casting complex values to real discards the imaginary part")

# Remains constant during a single run, can be calculated and loaded onto the controller beforehand.
# lambert(z, k) = lambert of zed for the kth branch
# gamma = float((-SMOOTHNESS_COEFFICIENT / (math.pow(EQUILIBRIUM_DISTANCE, 2) + SMOOTHNESS_COEFFICIENT))
#               * lambertw(-1 / (math.e * STEEPNESS_COEFFICIENT), k=-1))
gamma = 4.243528926663229
repulsive_gain = float(
    ATTRACTIVE_GAIN * ((1 - ATTRACTIVE_FRACTION) / (1 - REPULSIVE_FRACTION)) * math.sqrt(
        (math.e * STEEPNESS_COEFFICIENT * SMOOTHNESS_COEFFICIENT) / (gamma * math.exp(gamma))))

# repulsive_aoe = float(
#     2 * SMOOTHNESS_COEFFICIENT / lambertw(math.pow(math.e * STEEPNESS_COEFFICIENT * SMOOTHNESS_COEFFICIENT * (
#             (ATTRACTIVE_GAIN * (1 - ATTRACTIVE_FRACTION)) / (repulsive_gain * (1 - REPULSIVE_FRACTION))), 2),
#                                           k=0))
repulsive_aoe = 0.9480801932704002

attractive_aoe = repulsive_aoe
long_range_repulsive = ATTRACTIVE_FRACTION * ATTRACTIVE_GAIN
short_range_attractive = REPULSIVE_FRACTION * repulsive_gain

# TODO: Sliding gains and reaching gains remain constant per run? If not revert change and parameterize them again.

# SMC Gains
SATURATION_LEVEL = 1
# SLIDING_GAINS consists of [ Kx, Ky, Kz]
SLIDING_GAINS = numpy.array([3, 3, 20])

# REACHING_GAINS consists of [Cx, Cy, Cz]
REACHING_GAINS = numpy.array([1.5, 1.5, 1.5])

# TODO: For testing purposes...)
NODE_LOCATION = [[0, 0, 5]]

agents_vel = [[0, 0, 0]]  # TODO: is this true at t = 0;


# angles = [0, 0, 0]


def swarm_next_location(node_locations, agents_locations, agents_velocities) -> numpy.ndarray:
    """
    Swarm function

    go through each agent and compare positions with other quads
    find the difference between two locations
    find the norm (euclidean distance) between two agents


    :param node_locations: represents location of the target node represented as a matrix
    :param agents_locations: locations of the agents in an 2 dimensional array
    :param agents_velocities: velocities of the agents in an array
    """

    # Potential Field Gradient Calculation

    # Gradient of potential field
    dv = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT))  # create an array of values

    for agent_it_1 in range(AGENT_COUNT):
        # Inter-Agent Forces
        for agent_it_2 in range(AGENT_COUNT):
            n_x = numpy.linalg.norm(numpy.subtract(agents_locations[agent_it_1], agents_locations[agent_it_2]))

            for dimension_it in range(DIMENSION_COUNT):
                delta_x = agents_locations[agent_it_1][dimension_it] - agents_locations[agent_it_2][dimension_it]
                dv[agent_it_1][dimension_it] = dv[agent_it_1][dimension_it] - long_range_repulsive * (
                        delta_x / numpy.sqrt((pow(SMOOTHNESS_COEFFICIENT, 2)) + math.pow(n_x, 2))) - 2 * (
                                                       repulsive_gain / repulsive_aoe) * delta_x * numpy.exp(
                    (-math.pow(n_x, 2)) / repulsive_aoe)

        # Formation Attraction Forces
        if NODE_COUNT > 0:
            for node_it in range(NODE_COUNT):
                n_x = numpy.linalg.norm(numpy.subtract(agents_locations[agent_it_1],
                                                       node_locations[node_it]))  # norm of the vector between two bots
                for dimension_it in range(DIMENSION_COUNT):
                    delta_x = agents_locations[agent_it_1][dimension_it] - node_locations[node_it][dimension_it]
                    dv[agent_it_1][dimension_it] = dv[agent_it_1][dimension_it] + ATTRACTIVE_GAIN * (
                            delta_x / math.sqrt(math.pow(SMOOTHNESS_COEFFICIENT, 2) + (math.pow(n_x, 2)))) + (
                                                           short_range_attractive / attractive_aoe) * delta_x * \
                                                   math.exp((-math.pow(n_x, 2)) / attractive_aoe)
    sliding_surface = numpy.add(agents_velocities, dv)

    # Saturation Block [sat(s)]
    sx = numpy.zeros((8, 3))
    for agent_it_1 in range(AGENT_COUNT):
        for dimension_it in range(DIMENSION_COUNT):
            if abs(sliding_surface[agent_it_1][dimension_it]) > SATURATION_LEVEL:
                sx[agent_it_1][dimension_it] = numpy.sign(sliding_surface[agent_it_1][dimension_it]) * SATURATION_LEVEL
            else:
                sx[agent_it_1][dimension_it] = sliding_surface[agent_it_1][dimension_it]

    # Gains
    # Sliding Mode Control constants
    c = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT))
    k = numpy.zeros((AGENT_COUNT, DIMENSION_COUNT))

    # row by row multiplication
    for agent_it_1 in range(AGENT_COUNT):
        # TODO: what is the value of agents_velocities at t = 0
        c[agent_it_1] = numpy.multiply(agents_velocities[agent_it_1], REACHING_GAINS)
        k[agent_it_1] = numpy.multiply(sx[agent_it_1], SLIDING_GAINS)

    swarm_control_signal = k + c

    # print('Uxyz: ', swarm_control_signal)
    return swarm_control_signal


# Agent Positions
# Should come from GPSs
X1 = [0, 0, 0]
X2 = [0, 0, 0]
X3 = [0, 0, 0]
X4 = [0, 0, 0]
X5 = [0, 0, 0]
X6 = [0, 0, 0]
X7 = [0, 0, 0]
X8 = [0, 0, 0]
AGENT_LOC = numpy.array([X1])

AGENT_LOC = numpy.vstack([AGENT_LOC, X2])
AGENT_LOC = numpy.vstack([AGENT_LOC, X3])
AGENT_LOC = numpy.vstack([AGENT_LOC, X4])
AGENT_LOC = numpy.vstack([AGENT_LOC, X5])
AGENT_LOC = numpy.vstack([AGENT_LOC, X6])
AGENT_LOC = numpy.vstack([AGENT_LOC, X7])
AGENT_LOC = numpy.vstack([AGENT_LOC, X8])

# Node Initial positions
xad = [0, 0, 0]  # used to shift position of nodes
A1 = numpy.add([0, 0, 5], xad)
A2 = numpy.add([0, 4, 5], xad)
A3 = numpy.add([4, 20, 5], xad)
A4 = numpy.add([0, 20, 5], xad)
A5 = numpy.add([0, 0, 5], xad)
A6 = numpy.add([0, 0, 15], xad)
A7 = numpy.add([0, 0, 18], xad)
A8 = numpy.add([0, 0, 21], xad)
NODE_LOC = [A1, A2, A3, A4, A5, A6, A7, A8]

# XBEE Specific
XBEE_PORT = 'COM9'  # TODO: This will need to change most likely depending on what XBee is recognized as
BAUD_RATE = 9600
device = XBeeDevice(XBEE_PORT, BAUD_RATE)
device.open()
device.set_sync_ops_timeout(10)  # 10 second timeout
xbee_network = device.get_network()

# Array of Xbee MAC Addresses
# Index corresponds to index for swarm control result as well
QUAD_REMOTE_NODES_ID = ["REMOTE"]
QUAD_XBEE_DEVICES = numpy.empty(AGENT_COUNT, dtype=object)

for i in range(AGENT_COUNT):
    QUAD_XBEE_DEVICES[i] = xbee_network.discover_device(QUAD_REMOTE_NODES_ID[i])


def send_swarm_control(swarm_control_signal: numpy.ndarray):
    for i in range(AGENT_COUNT):
        swarm_control_signal[0][0] = 352233545.2139234904329012
        cur = numpy.array2string(swarm_control_signal[i], separator=',')
        # cur = swarm_control_signal[i]
        print(cur)
        # x = struct.pack("fff", cur[0], cur[1], cur[2])
        # print(x)
        device.send_data_broadcast(cur)


def update_agent_loc(xbee_message):
    """
    Callback function that handles updating the locations of a particular agent in
    :param xbee_message: represents the xyz coordinates of an agent
                         Will be in the format of:  agent_num, x, y, z
    :return nothing
    """
    xbee_data = xbee_message.data
    print(xbee_data)
    fmt = "iiii"  # this formatting should change depending on the type of GPS values i.e., longs doubles, etc.
    # print(fmt)
    print(list(struct.unpack(fmt, xbee_data)))
    tmp_arr = list(struct.unpack(fmt, xbee_message.data))
    # tmp_arr = [43.134620666503906, -70.93431854248047, 117.5]
    AGENT_LOC[tmp_arr[0]] = [tmp_arr[1], tmp_arr[2], tmp_arr[3]]


device.add_data_received_callback(update_agent_loc)

# TODO: Use a latch for each quad, once all latches are all unlocked
#

while True:
    # t1 = time.time()
    ret = swarm_next_location(NODE_LOC, AGENT_LOC, agents_vel)
    # retString = numpy.array2string(ret, separator=',')
    send_swarm_control(ret)
    # device.send_data_broadcast(retString)
    # time.sleep(5)
    # print(time.time() - t1)
