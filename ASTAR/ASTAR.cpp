#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <random>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std;

// 定义二维坐标点
struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}

    // 重载==运算符用于比较
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    // 重载<运算符用于排序
    bool operator<(const Point& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

// 为Point定义哈希函数
namespace std {
    template<>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

// 定义节点结构，用于A*算法
struct Node {
    Point pos;
    int g;  // 从起点到当前节点的实际距离
    int h;  // 启发式函数值（估计到终点的距离）
    int f;  // f = g + h
    Node* parent;

    Node(Point pos, int g, int h, Node* parent = nullptr)
        : pos(pos), g(g), h(h), f(g + h), parent(parent) {
    }

    // 重载>运算符用于优先队列
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// A*算法实现
class AStar {
private:
    vector<vector<int>> grid;  // 网格地图，0表示可通行，1表示障碍物
    int width, height;

public:
    AStar(int w, int h) : width(w), height(h) {
        grid.resize(height, vector<int>(width, 0));
    }

    // 随机生成障碍物
    void generateRandomObstacles(double obstacleRatio = 0.2) {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis(0.0, 1.0);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (dis(gen) < obstacleRatio) {
                    grid[y][x] = 1;  // 设置为障碍物
                }
            }
        }
    }

    // 随机生成起点和终点
    void generateRandomStartAndEnd(Point& start, Point& end) {
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<> disX(0, width - 1);
        uniform_int_distribution<> disY(0, height - 1);

        do {
            start = Point(disX(gen), disY(gen));
        } while (grid[start.y][start.x] == 1);  // 确保起点不是障碍物

        do {
            end = Point(disX(gen), disY(gen));
        } while ((end.x == start.x && end.y == start.y) || grid[end.y][end.x] == 1);
    }

    // 启发式函数（曼哈顿距离）
    int heuristic(const Point& a, const Point& b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }

    // 检查点是否在网格内且可通行
    bool isValid(const Point& p) {
        return p.x >= 0 && p.x < width && p.y >= 0 && p.y < height && grid[p.y][p.x] == 0;
    }

    // 获取相邻节点
    vector<Point> getNeighbors(const Point& p) {
        vector<Point> neighbors;
        vector<Point> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };  // 上下左右

        for (const auto& dir : directions) {
            Point neighbor(p.x + dir.x, p.y + dir.y);
            if (isValid(neighbor)) {
                neighbors.push_back(neighbor);
            }
        }

        return neighbors;
    }

    // A*算法主函数
    vector<Point> findPath(const Point& start, const Point& end) {
        // 优先队列（最小堆），存储待探索的节点
        priority_queue<Node, vector<Node>, greater<Node>> openSet;
        // 已探索的节点
        unordered_set<Point> closedSet;
        // 记录节点的g值
        vector<vector<int>> gValues(height, vector<int>(width, INT_MAX));

        // 初始化起点
        int h = heuristic(start, end);
        openSet.push(Node(start, 0, h));
        gValues[start.y][start.x] = 0;

        while (!openSet.empty()) {
            // 获取f值最小的节点
            Node current = openSet.top();
            openSet.pop();

            // 如果到达终点，回溯路径
            if (current.pos == end) {
                vector<Point> path;
                Node* node = &current;
                while (node != nullptr) {
                    path.push_back(node->pos);
                    node = node->parent;
                }
                reverse(path.begin(), path.end());
                return path;
            }

            // 将当前节点加入已探索集合
            closedSet.insert(current.pos);

            // 探索邻居节点
            for (const auto& neighborPos : getNeighbors(current.pos)) {
                // 如果邻居已经在已探索集合中，跳过
                if (closedSet.count(neighborPos)) continue;

                // 计算新的g值
                int tentativeG = current.g + 1;  // 假设每步代价为1

                // 如果找到更短的路径
                if (tentativeG < gValues[neighborPos.y][neighborPos.x]) {
                    gValues[neighborPos.y][neighborPos.x] = tentativeG;
                    int h = heuristic(neighborPos, end);
                    openSet.push(Node(neighborPos, tentativeG, h, new Node(current)));
                }
            }
        }

        // 没有找到路径
        return {};
    }

    // 打印地图和路径
    void printMap(const Point& start, const Point& end, const vector<Point>& path = {}) {
        unordered_set<Point> pathSet(path.begin(), path.end());

        cout << "Map (S=Start, E=End, #=Obstacle, .=Path, O=Open):\n";
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Point p(x, y);
                if (p == start) cout << "S ";
                else if (p == end) cout << "E ";
                else if (pathSet.count(p)) cout << ". ";
                else if (grid[y][x] == 1) cout << "# ";
                else cout << "O ";
            }
            cout << endl;
        }
    }
};

int main() {
    const int WIDTH = 15;
    const int HEIGHT = 10;

    AStar astar(WIDTH, HEIGHT);

    // 随机生成障碍物
    astar.generateRandomObstacles(0.25);  // 25%的障碍物密度

    // 随机生成起点和终点
    Point start, end;
    astar.generateRandomStartAndEnd(start, end);

    cout << "Start: (" << start.x << ", " << start.y << ")\n";
    cout << "End: (" << end.x << ", " << end.y << ")\n\n";

    // 打印初始地图
    astar.printMap(start, end);

    // 查找路径
    auto startTime = chrono::high_resolution_clock::now();
    vector<Point> path = astar.findPath(start, end);
    auto endTime = chrono::high_resolution_clock::now();

    // 输出结果
    if (!path.empty()) {
        cout << "\nPath found ("
            << chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count()
            << "ms):\n";
        astar.printMap(start, end, path);

        cout << "\nPath coordinates:\n";
        for (const auto& p : path) {
            cout << "(" << p.x << ", " << p.y << ") ";
        }
        cout << endl;
    }
    else {
        cout << "\nNo path found!" << endl;
    }

    return 0;
}

