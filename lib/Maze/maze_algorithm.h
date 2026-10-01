// maze_algorithm.h — chỉ phần thuật toán tìm đường (header-only, tự chứa).
// Chép file này vào xe khác (include/ hoặc lib/) rồi #include "maze_algorithm.h".
// Gồm: Cell, Point, ParentMaze với floodfill_update, get_next_move,
//      find_path_astar (có turn penalty), find_nearest_unvisited.
// Không dùng fstream/iostream nên build được trên ESP32/Arduino.
//
// Quy ước toạ độ: maze[x][y], y tăng về hướng Bắc, x tăng về hướng Đông.
// Thứ tự hướng: 0=N(y+1), 1=E(x+1), 2=S(y-1), 3=W(x-1).
#ifndef MAZE_ALGORITHM_H
#define MAZE_ALGORITHM_H

#include <vector>
#include <queue>
#include <algorithm>
#include <cstdint>
#include <cstdlib>

#ifndef MAZE_SIZE
#define MAZE_SIZE 16
#endif

struct Point {
    int8_t x;
    int8_t y;
};

class Cell {
public:
    int step = 10000;
    bool north_wall = false, south_wall = false, east_wall = false, west_wall = false;
    bool checked = false;
    bool run_visited = false; // ô đã đi qua
    bool known = false;

    void reset() {
        step = 10000;
        north_wall = south_wall = east_wall = west_wall = false;
        checked = run_visited = known = false;
    }
};

class ParentMaze {
public:
    Cell maze[MAZE_SIZE][MAZE_SIZE];

    ParentMaze() { add_boundary_walls(); }

    Cell& cell(int x, int y) { return maze[x][y]; }
    const Cell& cell(int x, int y) const { return maze[x][y]; }

    void add_boundary_walls() {
        for (int i = 0; i < MAZE_SIZE; i++) {
            maze[i][0].south_wall = true;
            maze[0][i].west_wall = true;
            maze[i][MAZE_SIZE - 1].north_wall = true;
            maze[MAZE_SIZE - 1][i].east_wall = true;
        }
    }

    void clear_mem() {
        for (int i = 0; i < MAZE_SIZE; i++)
            for (int j = 0; j < MAZE_SIZE; j++) maze[i][j].reset();
        add_boundary_walls();
    }

    // ---------------- Floodfill ----------------
    // BFS nhiều nguồn từ VÙNG ĐÍCH (gx..gx+size-1, gy..gy+size-1), ghi khoảng cách vào cell.step.
    // size = 1: đích 1 ô; size = 2: đích 2x2 (chuẩn micromouse, 4 ô trung tâm).
    // use_penalty: ô chưa đi qua bị cộng PENALTY (ưu tiên ô đã biết).
    // require_visited: chỉ cho đi qua ô đã run_visited.
    void floodfill_update(int goal_x, int goal_y, bool use_penalty, bool require_visited, int size = 1) {
        const int PENALTY = 30;
        const int QUEUE_SIZE = MAZE_SIZE * MAZE_SIZE;
        Point queue[QUEUE_SIZE];
        int head = 0, tail = 0;

        for (int y = 0; y < MAZE_SIZE; ++y)
            for (int x = 0; x < MAZE_SIZE; ++x) maze[x][y].step = 65535;

        for (int dx = 0; dx < size; ++dx)
            for (int dy = 0; dy < size; ++dy) {
                int gx = goal_x + dx, gy = goal_y + dy;
                if (gx < 0 || gx >= MAZE_SIZE || gy < 0 || gy >= MAZE_SIZE) continue;
                maze[gx][gy].step = 0;
                queue[tail] = {(int8_t)gx, (int8_t)gy}; tail = (tail + 1) % QUEUE_SIZE;
            }

        while (head != tail) {
            Point cur = queue[head]; head = (head + 1) % QUEUE_SIZE;
            int cur_step = maze[cur.x][cur.y].step;
            Point nb[4] = {{cur.x, (int8_t)(cur.y + 1)}, {(int8_t)(cur.x + 1), cur.y},
                           {cur.x, (int8_t)(cur.y - 1)}, {(int8_t)(cur.x - 1), cur.y}};
            bool walls[4] = {maze[cur.x][cur.y].north_wall, maze[cur.x][cur.y].east_wall,
                             maze[cur.x][cur.y].south_wall, maze[cur.x][cur.y].west_wall};
            for (int i = 0; i < 4; ++i) {
                if (walls[i]) continue;
                Point n = nb[i];
                if (n.x < 0 || n.x >= MAZE_SIZE || n.y < 0 || n.y >= MAZE_SIZE) continue;
                if (require_visited && !maze[n.x][n.y].run_visited) continue;
                int cost = 1;
                if (use_penalty && !maze[n.x][n.y].run_visited) cost += PENALTY;
                if (maze[n.x][n.y].step > cur_step + cost) {
                    maze[n.x][n.y].step = cur_step + cost;
                    queue[tail] = n; tail = (tail + 1) % QUEUE_SIZE;
                }
            }
        }
    }

    // Ô kề (không có tường) có step nhỏ nhất. Gọi sau floodfill_update.
    Point get_next_move(int cx, int cy) {
        int min_step = 65535;
        Point best = {(int8_t)cx, (int8_t)cy};
        Point nb[4] = {{(int8_t)cx, (int8_t)(cy + 1)}, {(int8_t)(cx + 1), (int8_t)cy},
                       {(int8_t)cx, (int8_t)(cy - 1)}, {(int8_t)(cx - 1), (int8_t)cy}};
        bool walls[4] = {maze[cx][cy].north_wall, maze[cx][cy].east_wall,
                         maze[cx][cy].south_wall, maze[cx][cy].west_wall};
        for (int i = 0; i < 4; ++i) {
            if (walls[i]) continue;
            Point n = nb[i];
            if (n.x < 0 || n.x >= MAZE_SIZE || n.y < 0 || n.y >= MAZE_SIZE) continue;
            if (maze[n.x][n.y].step < min_step) { min_step = maze[n.x][n.y].step; best = n; }
        }
        return best;
    }

    // ---------------- A* (có phạt rẽ) ----------------
    // unvisited_modifier: cộng thêm chi phí khi vào ô chưa đi qua.
    // forbid_unvisited: cấm hẳn ô chưa đi qua.
    // Trả về đường đi start..end (gồm cả 2 đầu); rỗng nếu không có đường.
    std::vector<Point> find_path_astar(Point start, Point end, int unvisited_modifier = 0,
                                       bool forbid_unvisited = false) {
        const int TURN_PENALTY = 10;
        const int MAX_COST = 65535;
        const int N = MAZE_SIZE;

        static int g_cost[MAZE_SIZE][MAZE_SIZE];
        static bool in_open[MAZE_SIZE][MAZE_SIZE];
        static bool closed[MAZE_SIZE][MAZE_SIZE];
        static Point predecessor[MAZE_SIZE][MAZE_SIZE];

        for (int x = 0; x < N; ++x)
            for (int y = 0; y < N; ++y) { g_cost[x][y] = MAX_COST; in_open[x][y] = false; closed[x][y] = false; }

        auto heuristic = [](const Point& a, const Point& b) { return std::abs(a.x - b.x) + std::abs(a.y - b.y); };

        g_cost[start.x][start.y] = 0;
        in_open[start.x][start.y] = true;
        predecessor[start.x][start.y] = start;

        const int MAX_ITER = N * N * 10;
        int iter = 0;

        while (true) {
            int best_f = MAX_COST;
            Point cur = {-1, -1};
            for (int x = 0; x < N; ++x)
                for (int y = 0; y < N; ++y)
                    if (in_open[x][y] && !closed[x][y]) {
                        int f = g_cost[x][y] + heuristic({(int8_t)x, (int8_t)y}, end);
                        if (f < best_f) { best_f = f; cur = {(int8_t)x, (int8_t)y}; }
                    }

            if (cur.x == -1) break; // không còn node mở -> không có đường
            if (cur.x == end.x && cur.y == end.y) {
                std::vector<Point> path;
                Point p = end;
                while (!(p.x == start.x && p.y == start.y)) {
                    path.push_back(p);
                    Point pred = predecessor[p.x][p.y];
                    if (pred.x == p.x && pred.y == p.y) break;
                    p = pred;
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            closed[cur.x][cur.y] = true;
            in_open[cur.x][cur.y] = false;

            Point nb[4] = {{cur.x, (int8_t)(cur.y + 1)}, {(int8_t)(cur.x + 1), cur.y},
                           {cur.x, (int8_t)(cur.y - 1)}, {(int8_t)(cur.x - 1), cur.y}};
            bool walls[4] = {maze[cur.x][cur.y].north_wall, maze[cur.x][cur.y].east_wall,
                             maze[cur.x][cur.y].south_wall, maze[cur.x][cur.y].west_wall};

            for (int i = 0; i < 4; ++i) {
                if (walls[i]) continue;
                Point n = nb[i];
                if (n.x < 0 || n.x >= N || n.y < 0 || n.y >= N) continue;
                if (forbid_unvisited && !maze[n.x][n.y].run_visited) continue;

                int cost = 1;
                if (!maze[n.x][n.y].run_visited) cost += unvisited_modifier;

                if (!(cur.x == start.x && cur.y == start.y)) {
                    Point pred = predecessor[cur.x][cur.y];
                    if ((cur.x - pred.x) != (n.x - cur.x) || (cur.y - pred.y) != (n.y - cur.y))
                        cost += TURN_PENALTY;
                }

                int tentative = g_cost[cur.x][cur.y] + cost;
                if (tentative < g_cost[n.x][n.y]) {
                    g_cost[n.x][n.y] = tentative;
                    predecessor[n.x][n.y] = cur;
                    if (!closed[n.x][n.y]) in_open[n.x][n.y] = true;
                }
            }

            if (++iter > MAX_ITER) break; // chống treo
        }
        return {};
    }

    // ---------------- BFS tìm ô chưa khám phá gần nhất ----------------
    // Trả về {-1,-1} nếu không còn ô nào tới được mà chưa biết vách (đã đi hết map).
    Point find_nearest_unvisited(Point start) {
        std::queue<Point> q;
        bool visited[MAZE_SIZE][MAZE_SIZE] = {};
        q.push(start); visited[start.x][start.y] = true;
        while (!q.empty()) {
            Point cur = q.front(); q.pop();
            // Ô cần khám phá = chưa đi qua VÀ chưa biết đủ 4 vách.
            // Ô đã nhìn thấy đủ vách từ ô bên cạnh (known) thì không cần bước vào nữa,
            // chỉ loang tiếp qua nó. Hết ô cần khám phá -> trả {-1,-1} -> xe dừng.
            if (!maze[cur.x][cur.y].run_visited && !maze[cur.x][cur.y].known) return cur;
            Point nb[4] = {{cur.x, (int8_t)(cur.y + 1)}, {(int8_t)(cur.x + 1), cur.y},
                           {cur.x, (int8_t)(cur.y - 1)}, {(int8_t)(cur.x - 1), cur.y}};
            bool walls[4] = {maze[cur.x][cur.y].north_wall, maze[cur.x][cur.y].east_wall,
                             maze[cur.x][cur.y].south_wall, maze[cur.x][cur.y].west_wall};
            for (int i = 0; i < 4; ++i) {
                if (walls[i]) continue;
                Point n = nb[i];
                if (n.x >= 0 && n.x < MAZE_SIZE && n.y >= 0 && n.y < MAZE_SIZE && !visited[n.x][n.y]) {
                    visited[n.x][n.y] = true; q.push(n);
                }
            }
        }
        return {-1, -1};
    }
};

#endif // MAZE_ALGORITHM_H
