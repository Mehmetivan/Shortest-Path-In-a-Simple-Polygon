import json
import math

# Triangle vertices
triangle_vertices = {
    0: [(151, 149), (201, 77), (318, 158)],
    1: [(151, 149), (318, 158), (378, 246)],
    2: [(378, 246), (386, 95), (447, 39)],
    3: [(378, 246), (447, 39), (540, 33)],
    4: [(378, 246), (540, 33), (623, 140)],
    5: [(623, 140), (648, 29), (770, 45)],
    6: [(623, 140), (770, 45), (800, 90)],
    7: [(378, 246), (623, 140), (800, 90)],
    8: [(875, 104), (930, 35), (1040, 102)],
    9: [(1040, 102), (1082, 8), (1167, 134)],
    10: [(875, 104), (1040, 102), (1167, 134)],
    11: [(875, 104), (1167, 134), (1122, 280)],
    12: [(1122, 280), (1028, 285), (971, 181)],
    13: [(875, 104), (1122, 280), (971, 181)],
    14: [(800, 90), (875, 104), (971, 181)],
    15: [(800, 90), (971, 181), (874, 271)],
    16: [(800, 90), (874, 271), (800, 235)],
    17: [(800, 90), (800, 235), (742, 147)],
    18: [(742, 147), (696, 251), (608, 323)],
    19: [(742, 147), (608, 323), (551, 297)],
    20: [(800, 90), (742, 147), (551, 297)],
    21: [(800, 90), (551, 297), (528, 202)],
    22: [(378, 246), (800, 90), (528, 202)],
    23: [(378, 246), (528, 202), (411, 336)],
    24: [(151, 149), (378, 246), (411, 336)],
    25: [(151, 149), (411, 336), (313, 319)],
    26: [(151, 149), (313, 319), (227, 244)],
    27: [(151, 149), (227, 244), (202, 340)],
    28: [(91, 72), (151, 149), (202, 340)],
    29: [(91, 72), (202, 340), (131, 395)],
    30: [(91, 72), (131, 395), (63, 337)],
    31: [(63, 337), (36, 205), (91, 72)],
}

# Adjacency list of triangles
triangle_adjacency = {
    0: [1],
    1: [0, 24],
    2: [3],
    3: [2, 4],
    4: [3, 7],
    5: [6],
    6: [5, 7],
    7: [4, 6, 22],
    8: [10],
    9: [10],
    10: [8, 9, 11],
    11: [10, 13],
    12: [13],
    13: [11, 12, 14],
    14: [13, 15],
    15: [14, 16],
    16: [15, 17],
    17: [16, 20],
    18: [19],
    19: [18, 20],
    20: [17, 19, 21],
    21: [20, 22],
    22: [7, 21, 23],
    23: [22, 24],
    24: [1, 23, 25],
    25: [24, 26],
    26: [25, 27],
    27: [26, 28],
    28: [27, 29],
    29: [28, 30],
    30: [29, 31],
    31: [30],
}

def compute_distance(p1, p2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def calculate_edge_weights(vertices, adjacency):
    """Calculate edge weights between connected triangles."""
    edge_weights = {}
    for tri1, neighbors in adjacency.items():
        edge_weights[tri1] = {}
        for tri2 in neighbors:
            shared_vertices = set(vertices[tri1]) & set(vertices[tri2])
            if len(shared_vertices) == 2:
                shared_vertices = list(shared_vertices)
                distance = compute_distance(shared_vertices[0], shared_vertices[1])
                edge_weights[tri1][tri2] = distance
    return edge_weights

# Calculate edge weights
edge_weights = calculate_edge_weights(triangle_vertices, triangle_adjacency)


with open("triangle_edge_weights.json", "w") as f:
    json.dump(edge_weights, f, indent=4)

print("Edge weights calculated and saved to triangle_edge_weights.json.")
