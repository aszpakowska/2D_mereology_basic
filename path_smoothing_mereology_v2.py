import matplotlib as plt
import matplotlib.pyplot as pypl
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math
import numpy as np
import pygame
from pygame.locals import *
import heapq
from typing import List, Tuple, Any, Dict, Set
import sys
import collections

# --- Initialization and Helpers ---

def read_coordinates(file_name):
    try:
        with open(file_name) as file:
            coordinates = []
            for line in file:
                row = line.split()
                if len(row) >= 2:
                    coordinates.append([float(row[0]), float(row[1])])
            return coordinates
    except FileNotFoundError:
        return [(0,0)]

def d_eucl(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def d_eucl_val(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def d_eucl2(x_temp, y_temp, x2_temp, y2_temp, goal_x, goal_y):
    dist1 = math.sqrt((x_temp - x2_temp)**2 + (y_temp - y2_temp)**2)
    dist2 = math.sqrt((x2_temp - goal_x)**2 + (y2_temp - goal_y)**2)
    return dist1 + 0.4 * dist2

def find_minimum_of_the_sum(list_of_coords):
    minimum = 1000000
    point = []
    for item in list_of_coords:
        sum_of_xy = item[0] + item[1]
        if sum_of_xy < minimum:
            minimum = sum_of_xy
            point = [[item[0], item[1]]]
    return point

def getImage(path, zoom=0.07):
    try:
        return OffsetImage(pypl.imread(path), zoom=zoom)
    except:
        return None

# --- Original Mereological Logic Blocks ---

def generate_mereological_field(goal_x, goal_y, obstacles, bounds):
    x_restriction_left, x_restriction, y_restriction_bottom, y_restriction = bounds
    f = []
    import collections
    q = collections.deque()
    
    # Ochrona przed wielokrotnym generowaniem krzywych dla tych samych rejonów (powodującym 137 Killed)
    visited = set()
    def is_visited(px, py):
        return (int(px/5), int(py/5)) in visited
    def mark_visited(px, py):
        visited.add((int(px/5), int(py/5)))

    def add_q(val):
        q.append(val)
        mark_visited(val[0], val[1])

    def pop_q():
        return q.popleft()

    add_q((goal_x, goal_y, 0))
    clockwise = True
    dist = 0
    
    previous_neighbours_list = []
    obstacles_coordinates_new = [(o[0], o[1]) for o in obstacles]
    
    def check_obstacle(obs_list, x_temp, y_temp):
        omit = 0
        for i in range(len(obs_list)):
            if d_eucl_val(x_temp, y_temp, obs_list[i][0], obs_list[i][1]) < 40:
                omit = 1
        return omit

    while len(q) > 0:
        if len(f) > 20000:
            print("Limit expanded blocks to 20000 to prevent OOM loop trap.")
            break
            
        previous_neighbours_list.clear()

        # Bug replication from final.py: intersection calculations effectively yielded 0 elements
        intersection1 = 0
        intersection2 = 0

        if intersection1 == 1 or intersection2 == 1:
            popped = pop_q()
            dist = dist + 0.5
            continue

        else:
            if clockwise == True:
                p0_x = q[0][0] - dist
                p0_y = q[0][1]
                p1_x = q[0][0] - dist
                p1_y = q[0][1] + dist
                p2_x = q[0][0]
                p2_y = q[0][1] + dist
                p3_x = q[0][0] + dist
                p3_y = q[0][1] + dist
                p4_x = q[0][0] + dist
                p4_y = q[0][1]
                p5_x = q[0][0] + dist
                p5_y = q[0][1] - dist
                p6_x = q[0][0]
                p6_y = q[0][1] - dist
                p8_x = q[0][0] - dist
                p8_y = q[0][1] - dist

                if len(q) == 1:
                    for pt in [(p0_x, p0_y, dist), (p1_x, p1_y, dist), (p2_x, p2_y, dist), 
                               (p3_x, p3_y, dist), (p4_x, p4_y, dist), (p5_x, p5_y, dist), 
                               (p6_x, p6_y, dist), (p8_x, p8_y, dist)]:
                        add_q(pt)
                    dist = 5
                else:
                    for x in reversed(q):
                        previous_neighbours_list.append(x)
                        if len(previous_neighbours_list) >= 8:
                            break

                    euclidean_dist1 = euclidean_dist2 = euclidean_dist3 = euclidean_dist4 = euclidean_dist5 = euclidean_dist6 = euclidean_dist7 = euclidean_dist8 = 0
                    
                    for i in previous_neighbours_list:
                        euclidean_dist1 = d_eucl_val(p0_x, p0_y, i[0], i[1])
                        euclidean_dist2 = d_eucl_val(p1_x, p1_y, i[0], i[1])
                        euclidean_dist3 = d_eucl_val(p2_x, p2_y, i[0], i[1])
                        euclidean_dist4 = d_eucl_val(p3_x, p3_y, i[0], i[1])
                        euclidean_dist5 = d_eucl_val(p4_x, p4_y, i[0], i[1])
                        euclidean_dist6 = d_eucl_val(p5_x, p5_y, i[0], i[1])
                        euclidean_dist7 = d_eucl_val(p6_x, p6_y, i[0], i[1])
                        euclidean_dist8 = d_eucl_val(p8_x, p8_y, i[0], i[1])

                    if p0_x <= x_restriction and p0_x >= x_restriction_left and p0_y <= y_restriction and p0_y >= y_restriction_bottom and euclidean_dist1 > 15:
                        if check_obstacle(obstacles_coordinates_new, p0_x, p0_y) == 0:
                            if not is_visited(p0_x, p0_y): 
                                add_q((p0_x, p0_y, dist))
                    if p1_x <= x_restriction and p1_x >= x_restriction_left and p1_y <= y_restriction and p1_y >= y_restriction_bottom and euclidean_dist2 > 15:
                        if check_obstacle(obstacles_coordinates_new, p1_x, p1_y) == 0:
                            if not is_visited(p1_x, p1_y): 
                                add_q((p1_x, p1_y, dist))
                    if p2_x <= x_restriction and p2_x >= x_restriction_left and p2_y <= y_restriction and p2_y >= y_restriction_bottom and euclidean_dist3 > 15:
                        if check_obstacle(obstacles_coordinates_new, p2_x-15, p2_y-15) == 0:
                            if not is_visited(p2_x, p2_y): 
                                add_q((p2_x, p2_y, dist))
                    if p3_x <= x_restriction and p3_x >= x_restriction_left and p3_y <= y_restriction and p3_y >= y_restriction_bottom and euclidean_dist4 > 15:
                        if check_obstacle(obstacles_coordinates_new, p3_x, p3_y) == 0:
                            if not is_visited(p3_x, p3_y): 
                                add_q((p3_x, p3_y, dist))
                    if p4_x <= x_restriction and p4_x >= x_restriction_left and p4_y <= y_restriction and p4_y >= y_restriction_bottom and euclidean_dist5 > 15:
                        if check_obstacle(obstacles_coordinates_new, p4_x, p4_y) == 0:
                            if not is_visited(p4_x, p4_y): 
                                add_q((p4_x, p4_y, dist))
                    if p5_x <= x_restriction and p5_x >= x_restriction_left and p5_y <= y_restriction and p5_y >= y_restriction_bottom and euclidean_dist6 > 15:
                        if check_obstacle(obstacles_coordinates_new, p5_x, p5_y) == 0:
                            if not is_visited(p5_x, p5_y): 
                                add_q((p5_x, p5_y, dist))
                    if p6_x <= x_restriction and p6_x >= x_restriction_left and p6_y <= y_restriction and p6_y >= y_restriction_bottom and euclidean_dist7 > 15:
                        if check_obstacle(obstacles_coordinates_new, p6_x, p6_y) == 0:
                            if not is_visited(p6_x, p6_y): 
                                add_q((p6_x, p6_y, dist))
                    if p8_x <= x_restriction and p8_x >= x_restriction_left and p8_y <= y_restriction and p8_y >= y_restriction_bottom and euclidean_dist8 > 15:
                        if check_obstacle(obstacles_coordinates_new, p8_x, p8_y) == 0:
                            if not is_visited(p8_x, p8_y): 
                                add_q((p8_x, p8_y, dist))

                clockwise = False
                popped = pop_q()
                f.append(popped)
                dist = dist + 1

            else:
                p0_x = q[0][0] - dist
                p0_y = q[0][1] - dist
                p1_x = q[0][0]
                p1_y = q[0][1] - dist
                p2_x = q[0][0] + dist
                p2_y = q[0][1] - dist
                p3_x = q[0][0] + dist
                p3_y = q[0][1]
                p4_x = q[0][0] + dist
                p4_y = q[0][1] + dist
                p5_x = q[0][0]
                p5_y = q[0][1] + dist
                p6_x = q[0][0] - dist
                p6_y = q[0][1] + dist
                p8_x = q[0][0] - dist
                p8_y = q[0][1]

                for x in reversed(q):
                    previous_neighbours_list.append(x)
                    if len(previous_neighbours_list) >= 8:
                        break

                euclidean_dist1 = euclidean_dist2 = euclidean_dist3 = euclidean_dist4 = euclidean_dist5 = euclidean_dist6 = euclidean_dist7 = euclidean_dist8 = 0
                for i in previous_neighbours_list:
                    euclidean_dist1 = d_eucl_val(p0_x, p0_y, i[0], i[1])
                    euclidean_dist2 = d_eucl_val(p1_x, p1_y, i[0], i[1])
                    euclidean_dist3 = d_eucl_val(p2_x, p2_y, i[0], i[1])
                    euclidean_dist4 = d_eucl_val(p3_x, p3_y, i[0], i[1])
                    euclidean_dist5 = d_eucl_val(p4_x, p4_y, i[0], i[1])
                    euclidean_dist6 = d_eucl_val(p5_x, p5_y, i[0], i[1])
                    euclidean_dist7 = d_eucl_val(p6_x, p6_y, i[0], i[1])
                    euclidean_dist8 = d_eucl_val(p8_x, p8_y, i[0], i[1])

                if p0_x <= x_restriction and p0_x >= x_restriction_left and p0_y <= y_restriction and p0_y >= y_restriction_bottom and euclidean_dist1 > 15:
                    if check_obstacle(obstacles_coordinates_new, p0_x, p0_y) == 0:
                        if not is_visited(p0_x, p0_y): 
                            add_q((p0_x, p0_y, dist))
                if p1_x <= x_restriction and p1_x >= x_restriction_left and p1_y <= y_restriction and p1_y >= y_restriction_bottom and euclidean_dist2 > 15:
                    if check_obstacle(obstacles_coordinates_new, p1_x, p1_y) == 0:
                        if not is_visited(p1_x, p1_y): 
                            add_q((p1_x, p1_y, dist))
                if p2_x <= x_restriction and p2_x >= x_restriction_left and p2_y <= y_restriction and p2_y >= y_restriction_bottom and euclidean_dist3 > 15:
                    if check_obstacle(obstacles_coordinates_new, p2_x, p2_y) == 0:
                        if not is_visited(p2_x, p2_y): 
                            add_q((p2_x, p2_y, dist))
                if p3_x <= x_restriction and p3_x >= x_restriction_left and p3_y <= y_restriction and p3_y >= y_restriction_bottom and euclidean_dist4 > 15:
                    if check_obstacle(obstacles_coordinates_new, p3_x, p3_y) == 0:
                        if not is_visited(p3_x, p3_y): 
                            add_q((p3_x, p3_y, dist))
                if p4_x <= x_restriction and p4_x >= x_restriction_left and p4_y <= y_restriction and p4_y >= y_restriction_bottom and euclidean_dist5 > 15:
                    if check_obstacle(obstacles_coordinates_new, p4_x, p4_y) == 0:
                        if not is_visited(p4_x, p4_y): 
                            add_q((p4_x, p4_y, dist))
                if p5_x <= x_restriction and p5_x >= x_restriction_left and p5_y <= y_restriction and p5_y >= y_restriction_bottom and euclidean_dist6 > 15:
                    if check_obstacle(obstacles_coordinates_new, p5_x, p5_y) == 0:
                        if not is_visited(p5_x, p5_y): 
                            add_q((p5_x, p5_y, dist))
                if p6_x <= x_restriction and p6_x >= x_restriction_left and p6_y <= y_restriction and p6_y >= y_restriction_bottom and euclidean_dist7 > 15:
                    if check_obstacle(obstacles_coordinates_new, p6_x, p6_y) == 0:
                        if not is_visited(p6_x, p6_y): 
                            add_q((p6_x, p6_y, dist))
                if p8_x <= x_restriction and p8_x >= x_restriction_left and p8_y <= y_restriction and p8_y >= y_restriction_bottom and euclidean_dist8 > 15:
                    if check_obstacle(obstacles_coordinates_new, p8_x, p8_y) == 0:
                        if not is_visited(p8_x, p8_y): 
                            add_q((p8_x, p8_y, dist))

                clockwise = True
                popped = pop_q()
                f.append(popped)
                dist = dist + 1


        if len(q) == 0:
            break
            
    return f

def search_path_original(start_x, start_y, goal_x, goal_y, field_list, iteration):
    import heapq
    from collections import defaultdict
    
    target_node = (goal_x, goal_y)
    start_node = (start_x, start_y)
    
    field_pts = [(int(pt[0]), int(pt[1])) for pt in field_list]
    field_set = set(field_pts)
    field_set.add(start_node)
    
    grid_lookup = defaultdict(list)
    cell_size = 20
    for px, py in field_set:
        grid_lookup[(int(px//cell_size), int(py//cell_size))].append((px, py))
        
    def get_neighbors(px, py):
        neighbors = []
        cx, cy = int(px//cell_size), int(py//cell_size)
        for dx in [-2, -1, 0, 1, 2]:
            for dy in [-2, -1, 0, 1, 2]:
                for nx, ny in grid_lookup[(cx+dx, cy+dy)]:
                    if nx == px and ny == py: continue
                    dist = d_eucl_val(px, py, nx, ny)
                    if dist <= 40:
                        neighbors.append((nx, ny, dist))
        return neighbors

    open_set = []
    heapq.heappush(open_set, (0, 0, start_node))
    came_from = {}
    g_score = {start_node: 0}
    
    closest_to_goal = start_node
    min_dist_to_goal = d_eucl_val(start_x, start_y, goal_x, goal_y)
    
    iters = 0
    while open_set and iters < 20000:
        iters += 1
        current_f, current_g, current = heapq.heappop(open_set)
        
        if current_g > g_score.get(current, float('inf')):
            continue
            
        dist_to_goal = d_eucl_val(current[0], current[1], goal_x, goal_y)
        if dist_to_goal < min_dist_to_goal:
            min_dist_to_goal = dist_to_goal
            closest_to_goal = current
            
        if dist_to_goal <= 20:
            closest_to_goal = current
            break
            
        for nx, ny, step_cost in get_neighbors(current[0], current[1]):
            neighbor = (nx, ny)
            tentative_g = g_score[current] + step_cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + d_eucl_val(nx, ny, goal_x, goal_y)
                heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                
    path = []
    curr = closest_to_goal
    while curr in came_from:
        path.append(curr)
        curr = came_from[curr]
    path.append(start_node)
    path.reverse()
    
    return path

def search_min_distances_original(path, goal_coord):
    path_distances = []
    optimal_path = []
    for i in range(len(path)):
        euclid_dist = d_eucl_val(path[i][0], path[i][1], goal_coord[0], goal_coord[1])
        if i == 0:
            path_distances.append(euclid_dist)
            optimal_path.append(path[i])
        else:
            if euclid_dist < path_distances[-1]:
                optimal_path.append(path[i])
                path_distances.append(euclid_dist)
            if euclid_dist == path_distances[-1] and i + 1 < len(path) - 2:
                euclid_superset = d_eucl_val(path[i+1][0], path[i+1][1], goal_coord[0], goal_coord[1])
                if euclid_superset < euclid_dist:
                    optimal_path.pop(-1)
                    optimal_path.append(path[i])
                    path_distances.append(euclid_superset)
                if euclid_superset == euclid_dist and i + 2 < len(path) - 2:
                    euclid_superset2 = d_eucl_val(path[i+2][0], path[i+2][1], goal_coord[0], goal_coord[1])
                    if euclid_superset2 < euclid_dist:
                        optimal_path.pop(-1)
                        optimal_path.append(path[i])
                        path_distances.append(euclid_superset2)
                else:
                    continue
    return optimal_path

def path_smoothing_original(path, iter_number, obstacles):
    if len(path) < 3: return path
    x_list = [p[0] for p in path]
    y_list = [p[1] for p in path]
    x0 = x_list.copy()
    y0 = y_list.copy()
    alpha = 0.5
    beta = 0.1
    
    # Zwiększenie iteracji by wygładzenie było widoczne przy dużej liczbie punktów (np. 15 -> 100)
    iter_number = max(iter_number, 100) 
    
    new_path = []
    for i0 in range(0, iter_number):
        for i in range(1, len(path) - 1):
            nx = x_list[i] + alpha * (x_list[i - 1] + x_list[i + 1] - 2 * x_list[i])
            ny = y_list[i] + alpha * (y_list[i - 1] + y_list[i + 1] - 2 * y_list[i])
            
            nx = nx + beta * (x0[i] - nx)
            ny = ny + beta * (y0[i] - ny)
            
            # Anti-collision check during smoothing
            col = False
            for ox, oy in obstacles:
                if math.sqrt((nx - ox)**2 + (ny - oy)**2) < 35: # safety smoothing margin
                    col = True
                    break
            
            if not col:
                x_list[i] = nx
                y_list[i] = ny
            
        if i0 == iter_number - 1:
            new_path = list(zip(x_list, y_list))
    return new_path

# --- Main Setup and UI ---

# 1. Load Data
goal_coordianets = [(250, 70)] 
center_list = read_coordinates('old_txt/txt_files5/boundaries_detector.txt')
obstacles_coordinates = [(250, 250), (250, 300)]
start_coordinates = [(250, 400)]

# 2. Normalize
mins = find_minimum_of_the_sum(center_list)
x_min, y_min = mins[0][0], mins[0][1] if mins else (0,0)

x_raw = [x for x, y in center_list]
y_raw = [y for x, y in center_list]
padding = 60
width = int(max(x_raw) - x_min + 2 * padding)
height = int(max(y_raw) - y_min + 2 * padding)

def transform(p):
    return (int(p[0] - x_min + padding), int(height - (p[1] - y_min + padding)))

start_node = transform(start_coordinates[0])
goal_node = transform(goal_coordianets[0])
border_p = [transform(p) for p in center_list]
obs_p_initial = [transform(p) for p in obstacles_coordinates]

# Matplotlib Background
dpi = 100
fig = pypl.figure(figsize=(width/dpi, height/dpi), dpi=dpi)
ax = fig.add_axes([0, 0, 1, 1])
marker_paths = ['marker_images/marker_1751.png', 'marker_images/marker_4076.png', 
                'marker_images/marker_1281.png', 'marker_images/marker_1184.png']
goal_path = 'marker_images/marker_497.png'
start_path = 'start_images/start_point.jpg'

for i, p in enumerate(border_p):
    if i < len(marker_paths): 
        img = getImage(marker_paths[i])
        if img: ax.add_artist(AnnotationBbox(img, p, frameon=False))

ax.add_artist(AnnotationBbox(getImage(goal_path), transform(goal_coordianets[0]), frameon=False))
ax.add_artist(AnnotationBbox(getImage(start_path), transform(start_coordinates[0]), frameon=False))

ax.set_xlim(0, width)
ax.set_ylim(height, 0)
ax.axis('off')
pypl.savefig('plots/my_plot_mereology.png', dpi=dpi)
pypl.close(fig)

# 3. Pygame UI
pygame.init()
bg_surf = pygame.image.load('plots/my_plot_mereology.png')
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Exact Mereology Potential Fields")

current_obstacles = set(obs_p_initial)
field_surf = pygame.Surface((width, height), pygame.SRCALPHA)

def get_full_results(obs_set):
    # Check if start or goal are blocked
    start_blocked = any(d_eucl_val(start_node[0], start_node[1], o[0], o[1]) < 30 for o in obs_set)
    goal_blocked = any(d_eucl_val(goal_node[0], goal_node[1], o[0], o[1]) < 30 for o in obs_set)
    
    if start_blocked or goal_blocked:
        return "ERROR", []
        
    obs_list = list(obs_set)
    
    # Calculate exact boundaries from borders
    x_res_left = min([p[0] for p in border_p])
    x_res = max([p[0] for p in border_p])
    y_res_bottom = min([p[1] for p in border_p])
    y_res = max([p[1] for p in border_p])
    bounds = (x_res_left, x_res, y_res_bottom, y_res)

    f_field = generate_mereological_field(goal_node[0], goal_node[1], obs_list, bounds)
    
    raw_path = search_path_original(start_node[0], start_node[1], goal_node[0], goal_node[1], f_field, 3000)
    # search_min_distances_original trimmed valid detours around obstacles
    # optimal = search_min_distances_original(raw_path, goal_node)
    smoothed = path_smoothing_original(raw_path, 15, obs_list)
    
    return smoothed, f_field

def redraw_field(f_list):
    field_surf.fill((0,0,0,0))
    for item in f_list:
        # Original 30x30 rectangles
        pygame.draw.rect(field_surf, (100, 100, 100, 120), (int(item[0]-15), int(item[1]-15), 30, 30), 1)

clock = pygame.time.Clock()
run = True
update_count = 0
pygame.font.init()
font = pygame.font.SysFont('Arial', 24)

# Load Obstacle Images
obs_paths = ['marker_images/marker_733.png', 'marker_images/marker_2165.png']
obs_images = []
for p in obs_paths:
    try:
        img = pygame.image.load(p).convert_alpha()
        img = pygame.transform.scale(img, (40, 40))
        obs_images.append(img)
    except:
        pass

print("Calculating Mereological Potential Field...")
current_path, current_field = get_full_results(current_obstacles)
redraw_field(current_field)

while run:
    screen.blit(bg_surf, (0, 0))
    screen.blit(field_surf, (0, 0))
    
    # Draw Obstacles
    for obs in current_obstacles:
        if obs in obs_p_initial:
            idx = obs_p_initial.index(obs)
            if obs_images and idx < len(obs_images):
                img = obs_images[idx]
                screen.blit(img, (obs[0] - img.get_width()//2, obs[1] - img.get_height()//2))
            else:
                pygame.draw.rect(screen, (200, 0, 0), (obs[0]-15, obs[1]-15, 30, 30))
        else:
            # Nowe przeszkody to zwykłe kwadraty
            pygame.draw.rect(screen, (200, 0, 0), (obs[0]-15, obs[1]-15, 30, 30))

    if current_path == "ERROR":
        warning_surf = font.render("Błąd: Start lub cel w obszarze przeszkody!", True, (255, 0, 0))
        screen.blit(warning_surf, (20, 50))
    elif not current_path and d_eucl_val(start_node[0], start_node[1], goal_node[0], goal_node[1]) > 10:
        # If no path found or start/goal blocked
        pass
        
    elif isinstance(current_path, list) and len(current_path) > 1:
        points = [(int(x), int(y)) for x, y in current_path]
        if len(points) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, points, 5)

    pygame.draw.circle(screen, (255, 0, 0), start_node, 5)
    pygame.draw.circle(screen, (255, 0, 0), goal_node, 5)

    text_surf = font.render(f"Liczba aktualizacji: {update_count}", True, (255, 255, 0))
    screen.blit(text_surf, (20, 20))

    for event in pygame.event.get():
        if event.type == QUIT:
            run = False
        if event.type == MOUSEBUTTONDOWN:
            m_pos = pygame.mouse.get_pos()
            changed = False
            if event.button == 1:
                current_obstacles.add(m_pos)
                changed = True
            elif event.button == 3:
                to_remove = [o for o in current_obstacles if d_eucl_val(m_pos[0], m_pos[1], o[0], o[1]) < 30]
                for r in to_remove:
                    current_obstacles.remove(r)
                    changed = True
            
            if changed:
                new_path, new_field = get_full_results(current_obstacles)
                if current_path != new_path:
                    update_count += 1
                current_path = new_path
                current_field = new_field
                redraw_field(current_field)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
