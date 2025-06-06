import math

set_max_path_size(20)
set_node_cost(50)

team_names = ('yellow', 'blue')  # (left side, right side)

nodes = {}
vertices = []

# Substitue patterns with team names
# The following patterns are replaced:
#   {t}  team name
#   {o}  other team name
def format_team_pattern(s, side):
    if side == 0:
        return s.replace('{t}', team_names[0]).replace('{o}', team_names[1])
    else:
        return s.replace('{t}', team_names[1]).replace('{o}', team_names[0])

def add_team_elements(team_nodes, team_vertices):
    for name, (x, y) in team_nodes.items():
        nodes[format_team_pattern(name, 0)] = (-x, y)
        nodes[format_team_pattern(name, 1)] = (x, y)
    for name in team_vertices:
        vertices.append(format_team_pattern(name, 0))
        vertices.append(format_team_pattern(name, 1))

def auto_node_name(x, y):
    if x == 0:
        return "xy_%s_%s" % (x, y)
    elif x < 0:
        return "xy_%s_%s_%s" % (team_names[0], -x, y)
    else:
        return "xy_%s_%s_%s" % (team_names[1], x, y)

def distance(xy0, xy1):
    return math.sqrt((xy0[0]-xy1[0])**2 + (xy0[1]-xy1[1])**2)

def distance_segment(o, a, b):
    """Compute distance from point o to segment [ab]"""
    xo, yo = o
    xa, ya = a
    xb, yb = b
    xab, yab = xb-xa, yb-ya

    # compute the projected p of o on ab
    dap = xab*(xo-xa) + yab*(yo-ya)
    u = dap / distance(a, b)
    if u <= 0:
        return distance(o, a)
    elif u >= 1:
        return distance(o, b)
    else:
        return distance(o, (dap * xab + xa, dap * yab + yb))


def find_closest_node(nodes, xy0):
    return min(nodes.items(), key=lambda n_xy: distance(n_xy[1], xy0))[0]

def add_triangle_grid(x1, y0, y1, r, obstacles):

    # Iterate on possible node positions.
    # Shift y depending on x parity.
    #
    # 2 o---o---o
    #    \ / \ /
    # 1 --o---o--
    #    / \ / \
    # 0 o---o---o
    #   0 1 2 3 4

    grid_node_names = {}  # {(ix, iy): name}
    new_nodes = {}
    dx, dy = r/2, r * math.sin(math.pi/3)
    ix_max = int(x1 / dx)
    iy_max = int((y1-y0) / dy)
    for ix in range(-ix_max, ix_max+1):
        for iy in range(0, iy_max+1):
            # skip points in the middle of horizonal sides
            if (ix + iy) % 2 == 1:
                continue
            xy = (int(ix*dx), int(y0+iy*dy))
            ixy = ix, iy
            # skip positions blocked by an obstacle
            if any(distance(xy, xyo) < ro for xyo, ro in obstacles):
                continue
            # skip positions close to an existing node
            if nodes and distance(nodes[find_closest_node(nodes, xy)], xy) < r/2:
                continue
            name = auto_node_name(*xy)
            new_nodes[name] = xy
            grid_node_names[ixy] = name

    # add vertices for grid nodes close to existing nodes
    for name0, xy0 in new_nodes.items():
        for name1, xy1 in nodes.items():
            if distance(xy0, xy1) < 1.5 * r:
                if not any(distance_segment(xyo, xy0, xy1) < ro for xyo, ro in obstacles):
                    vertices.append((name0, name1))

    # merge the nodes (after all uses of the original `nodes`)
    nodes.update(new_nodes)

    # Add vertices for all grid nodes
    # close to each other.              \ /
    # Check vertices above and right:    o---
    for ix in range(-ix_max, ix_max+1):
        for iy in range(0, iy_max+1):
            name0 = grid_node_names.get((ix, iy))
            if name0 is None:
                continue
            for dix, diy in ((-1, 1), (1, 1), (2, 0)):
                name1 = grid_node_names.get((ix + dix, iy + diy))
                if name1 is None:
                    continue
                xy0, xy1 = nodes[name0], nodes[name1]
                if not any(distance_segment(xyo, xy0, xy1) < ro for xyo, ro in obstacles):
                    vertices.append((name0, name1))


# Definition of nodes and vertices
# Team elements are given for right side team (X>0)


add_team_elements({
    'stage':(0,1200),
    'center_bot':(0,700),
    'center_top':(0,1000),
    '{t}_backstage': (1200, 1300),
    '{t}_main_start': (300, 600),
    '{t}_alt_start': (-900, 900),
    '{t}_reserved_stock': (700, 1300),
    '{t}_central_stock_top': (400,1250 ),
    '{t}_central_stock_bot': (400,700 ),
    '{t}_side_stock_top': (1200, 1200),
    '{t}_side_stock_bot': (1200, 400),
    '{t}_bot_stock': (700, 500),
    '{t}_construction_area': (700, 450),
    '{t}_side_path_bot': (800,800),
    '{t}_side_path_top': (800,1100),
}, [
    '{t}_main_start {t}_side_path_bot {t}_side_path_top {t}_reserved_stock {t}_backstage',
    '{t}_alt_start {t}_main_start',
    'center_bot stage',
])

# make sure the grid does not overlap the stage
add_triangle_grid(1200, 450, 1400, 300, [
    # central stocks
    ((520, 950), 200),
    ((300, 950), 200),
    ((-520, 950),200),
    ((-300, 950),200),
])

add_graph('pathfinding_graph', nodes, vertices)

