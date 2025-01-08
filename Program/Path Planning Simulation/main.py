import turtle
import time

# Constants for grid size and cell dimensions
grid_width = 8
grid_height = 8
cell_size = 40

# Grid definition (0: free cell, 1: obstacle)
grid = [
    [0, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 1, 0, 1, 0],
    [0, 1, 0, 1, 1, 0, 1, 0],
    [1, 1, 0, 1, 1, 0, 1, 1],
    [1, 1, 0, 1, 1, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 1, 1],
]

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

def draw_grid():
    turtle.speed(0)
    turtle.hideturtle()
    for i in range(grid_width):
        for j in range(grid_height):
            x = i * cell_size - (grid_width // 2) * cell_size
            y = (grid_height // 2) * cell_size - j * cell_size
            draw_cell(x, y, "#ddd" if grid[i][j] == 0 else "#444")

def draw_cell(x, y, color):
    turtle.penup()
    turtle.goto(x, y)
    turtle.pendown()
    turtle.fillcolor(color)
    turtle.begin_fill()
    for _ in range(4):
        turtle.forward(cell_size)
        turtle.right(90)
    turtle.end_fill()

def draw_path(path):
    for point in path:
        x = point.x * cell_size - (grid_width // 2) * cell_size
        y = (grid_height // 2) * cell_size - point.y * cell_size
        draw_cell(x, y, "#0f0")
        time.sleep(0.1)

def draw_start_goal(start, goal):
    start_x = start.x * cell_size - (grid_width // 2) * cell_size
    start_y = (grid_height // 2) * cell_size - start.y * cell_size
    goal_x = goal.x * cell_size - (grid_width // 2) * cell_size
    goal_y = (grid_height // 2) * cell_size - goal.y * cell_size

    draw_cell(start_x, start_y, "#00f")
    draw_cell(goal_x, goal_y, "#f00")

def a_star(start, goal):
    open_set = [start]
    came_from = {}
    g_score = { (x, y): float('inf') for x in range(grid_width) for y in range(grid_height) }
    g_score[(start.x, start.y)] = 0
    f_score = { (x, y): float('inf') for x in range(grid_width) for y in range(grid_height) }
    f_score[(start.x, start.y)] = heuristic(start, goal)

    while open_set:
        current = min(open_set, key=lambda p: f_score[(p.x, p.y)])
        if current.x == goal.x and current.y == goal.y:
            return reconstruct_path(came_from, current)

        open_set.remove(current)

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = Point(current.x + dx, current.y + dy)
            if 0 <= neighbor.x < grid_width and 0 <= neighbor.y < grid_height and grid[neighbor.x][neighbor.y] == 0:
                tentative_g_score = g_score[(current.x, current.y)] + 1
                if tentative_g_score < g_score[(neighbor.x, neighbor.y)]:
                    came_from[(neighbor.x, neighbor.y)] = current
                    g_score[(neighbor.x, neighbor.y)] = tentative_g_score
                    f_score[(neighbor.x, neighbor.y)] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.append(neighbor)

    return []

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[(current.x, current.y)]
    path.append(current)
    return path[::-1]

def main():
    turtle.setup(width=800, height=800)
    turtle.tracer(0, 0)

    start = Point(0, 0)
    goal = Point(7, 5)

    draw_grid()
    draw_start_goal(start, goal)
    turtle.update()

    path = a_star(start, goal)

    if path:
        draw_path(path)
        print("Path found!")
    else:
        print("No path found.")

    turtle.done()

if __name__ == "__main__":
    main()
