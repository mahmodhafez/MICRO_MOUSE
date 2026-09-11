#include "FloodFill.h"
#include "API.h"

#include <iostream>
#include <queue>
#include <vector>
#include <climits>
#include <cmath>

// Direction offsets: NORTH (0), EAST (1), SOUTH (2), WEST (3)
static const int DX[4] = {0, 1, 0, -1};
static const int DY[4] = {1, 0, -1, 0};

FloodFill::FloodFill()
    : m_x(0),
      m_y(0),
      m_heading(NORTH) {
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            for (int d = 0; d < 4; ++d) {
                m_walls[x][y][d] = false;
            }
            m_dist[x][y] = -1;
            m_displayedDist[x][y] = -2;
            m_visited[x][y] = false;
        }
    }
}

void FloodFill::log(const std::string& message) const {
    std::cerr << "[FloodFill] " << message << std::endl;
}

char FloodFill::directionToChar(Direction dir) const {
    switch (dir) {
        case NORTH: return 'n';
        case EAST:  return 'e';
        case SOUTH: return 's';
        case WEST:  return 'w';
    }
    return 'n';
}

bool FloodFill::isInside(int x, int y) const {
    return x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT;
}

bool FloodFill::isGoalCell(int x, int y) const {
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

bool FloodFill::isAtGoal() const {
    return isGoalCell(m_x, m_y);
}

void FloodFill::addWall(int x, int y, Direction dir) {
    if (!isInside(x, y) || m_walls[x][y][dir]) {
        return;
    }

    // Mark physical wall on cell
    m_walls[x][y][dir] = true;
    API::setWall(x, y, directionToChar(dir));

    // Mirror to adjacent neighbor
    int nx = x + DX[dir];
    int ny = y + DY[dir];
    if (isInside(nx, ny)) {
        Direction opp = static_cast<Direction>((dir + 2) % 4);
        m_walls[nx][ny][opp] = true;
        API::setWall(nx, ny, directionToChar(opp));
    }
}

void FloodFill::init() {
    log("Initializing optimized FloodFill controller...");

    m_x = 0;
    m_y = 0;
    m_heading = NORTH;

    // 1. Set outer boundary walls
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        addWall(x, 0, SOUTH);
        addWall(x, MAZE_HEIGHT - 1, NORTH);
    }
    for (int y = 0; y < MAZE_HEIGHT; ++y) {
        addWall(0, y, WEST);
        addWall(MAZE_WIDTH - 1, y, EAST);
    }

    // 2. Mark start cell
    m_visited[0][0] = true;
    API::setColor(0, 0, 'G');

    // 3. Dynamically sense walls at (0, 0) using physical sensors
    senseAndUpdateWalls();

    // 5. Initial BFS flood fill
    recalculateDistances();
    updateDisplayedTexts();

    log("Initialization complete. Start cell: (0, 0), Heading: NORTH.");
}

bool FloodFill::senseAndUpdateWalls() {
    bool wallChanged = false;

    Direction frontDir = m_heading;
    Direction rightDir = static_cast<Direction>((m_heading + 1) % 4);
    Direction leftDir  = static_cast<Direction>((m_heading + 3) % 4);

    if (API::wallFront() && !m_walls[m_x][m_y][frontDir]) {
        addWall(m_x, m_y, frontDir);
        wallChanged = true;
    }
    if (API::wallRight() && !m_walls[m_x][m_y][rightDir]) {
        addWall(m_x, m_y, rightDir);
        wallChanged = true;
    }
    if (API::wallLeft() && !m_walls[m_x][m_y][leftDir]) {
        addWall(m_x, m_y, leftDir);
        wallChanged = true;
    }

    return wallChanged;
}

void FloodFill::recalculateDistances() {
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            m_dist[x][y] = -1;
        }
    }

    std::queue<std::pair<int, int>> q;
    const std::vector<std::pair<int, int>> goals = {
        {7, 7}, {7, 8}, {8, 7}, {8, 8}
    };

    for (const auto& g : goals) {
        m_dist[g.first][g.second] = 0;
        q.push(g);
    }

    // BFS flood across all open cells
    while (!q.empty()) {
        auto current = q.front();
        q.pop();

        int cx = current.first;
        int cy = current.second;
        int currentDist = m_dist[cx][cy];

        for (int d = 0; d < 4; ++d) {
            if (!m_walls[cx][cy][d]) {
                int nx = cx + DX[d];
                int ny = cy + DY[d];

                if (isInside(nx, ny) && m_dist[nx][ny] == -1) {
                    m_dist[nx][ny] = currentDist + 1;
                    q.push({nx, ny});
                }
            }
        }
    }
}

void FloodFill::updateDisplayedTexts() {
    for (int x = 0; x < MAZE_WIDTH; ++x) {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            int d = m_dist[x][y];
            if (d != m_displayedDist[x][y]) {
                m_displayedDist[x][y] = d;
                if (d >= 0) {
                    API::setText(x, y, std::to_string(d));
                } else {
                    API::setText(x, y, "X");
                }
            }
        }
    }
}

void FloodFill::checkAndSealDeadEnd(int x, int y) {
    // Never seal start cell, goal cells, or the cell currently occupied by mouse
    if ((x == 0 && y == 0) || isGoalCell(x, y) || (x == m_x && y == m_y)) {
        return;
    }

    int wallCount = 0;
    Direction openDir = NORTH;
    for (int d = 0; d < 4; ++d) {
        if (m_walls[x][y][d]) {
            wallCount++;
        } else {
            openDir = static_cast<Direction>(d);
        }
    }

    // If it has 3 walls, it is a confirmed dead end
    if (wallCount == 3) {
        addWall(x, y, openDir);
        API::setColor(x, y, 'k'); // Visual marker: dark gray for pruned dead-end

        // Cascade backward along the corridor if the adjacent cell now also has 3 walls
        int nx = x + DX[openDir];
        int ny = y + DY[openDir];
        if (isInside(nx, ny) && (nx != m_x || ny != m_y)) {
            checkAndSealDeadEnd(nx, ny);
        }
    }
}

Direction FloodFill::getBestNextDirection() {
    int bestScore = INT_MAX;
    Direction bestDir = m_heading;

    for (int d = 0; d < 4; ++d) {
        // Physical wall must NOT exist
        if (!m_walls[m_x][m_y][d]) {
            int nx = m_x + DX[d];
            int ny = m_y + DY[d];

            if (isInside(nx, ny) && m_dist[nx][ny] >= 0) {
                int neighborDist = m_dist[nx][ny];
                int turnDiff = (d - m_heading + 4) % 4;
                int turns = (turnDiff == 3) ? 1 : turnDiff; // 0=straight, 1=turn, 2=turnaround

                // Turn penalty:
                // In MMS score, 1 turn costs 1.6 distance units!
                // Straight: 0, 90-deg turn: 5, Turnaround: 15
                int score = neighborDist * 10;
                if (turns == 1) score += 5;
                else if (turns == 2) score += 15;

                // Unvisited cell bonus: encourages exploring new paths rather than re-traversing
                if (!m_visited[nx][ny]) {
                    score -= 2;
                }

                // Center proximity tie-breaker: slight bias towards maze center
                double centerDist = std::abs(nx - 7.5) + std::abs(ny - 7.5);
                score += static_cast<int>(centerDist * 0.3);

                if (score < bestScore) {
                    bestScore = score;
                    bestDir = static_cast<Direction>(d);
                }
            }
        }
    }

    // Safety fallback: if no neighbor has valid flood distance, pick any open side to avoid crash
    if (bestScore == INT_MAX) {
        for (int d = 0; d < 4; ++d) {
            if (!m_walls[m_x][m_y][d]) {
                int nx = m_x + DX[d];
                int ny = m_y + DY[d];
                if (isInside(nx, ny)) {
                    bestDir = static_cast<Direction>(d);
                    break;
                }
            }
        }
    }

    return bestDir;
}

void FloodFill::turnTo(Direction targetHeading) {
    int diff = (targetHeading - m_heading + 4) % 4;

    if (diff == 1) {
        API::turnRight();
    } else if (diff == 2) {
        API::turnRight();
        API::turnRight();
    } else if (diff == 3) {
        API::turnLeft();
    }

    m_heading = targetHeading;
}

void FloodFill::moveOneCell() {
    API::moveForward();
    m_x += DX[m_heading];
    m_y += DY[m_heading];
}

bool FloodFill::step() {
    if (isAtGoal()) {
        log("Goal reached at (" + std::to_string(m_x) + ", " + std::to_string(m_y) + ")!");
        highlightShortestPath();
        return false;
    }

    // 1. Choose best direction (turn-penalized)
    Direction nextDir = getBestNextDirection();

    // 2. Turn to face target direction
    turnTo(nextDir);

    // 3. Mark cell being left as VISITED (BLUE)
    API::setColor(m_x, m_y, 'B');

    int prevX = m_x;
    int prevY = m_y;

    // 4. Move forward one cell
    moveOneCell();

    // 5. Mark entered cell as CURRENT (GREEN)
    API::setColor(m_x, m_y, 'G');
    m_visited[m_x][m_y] = true;

    // 6. Seal dead end behind us if the previous cell had 3 physical walls
    checkAndSealDeadEnd(prevX, prevY);

    // 7. Sense physical walls at new cell
    bool wallsChanged = senseAndUpdateWalls();

    // 8. If walls changed, recalculate flood fill distances
    if (wallsChanged) {
        recalculateDistances();
        updateDisplayedTexts();
    }

    if (isAtGoal()) {
        log("Goal reached at (" + std::to_string(m_x) + ", " + std::to_string(m_y) + ")!");
        highlightShortestPath();
        return false;
    }

    return true;
}

void FloodFill::highlightShortestPath() {
    log("Reconstructing and highlighting the shortest path from start (0, 0) to goal...");

    int currX = 0;
    int currY = 0;

    if (m_dist[currX][currY] <= 0 && !isGoalCell(currX, currY)) {
        log("Cannot reconstruct shortest path: start distance invalid.");
        return;
    }

    Direction lastDir = NORTH;

    while (!isGoalCell(currX, currY)) {
        if (currX != m_x || currY != m_y) {
            API::setColor(currX, currY, 'Y');
        }

        int currentDist = m_dist[currX][currY];
        int nextX = -1;
        int nextY = -1;
        Direction nextDir = lastDir;

        // Prefer continuing straight along lastDir to minimize turns
        for (int i = 0; i < 4; ++i) {
            int d = (lastDir + i) % 4;
            if (!m_walls[currX][currY][d]) {
                int nx = currX + DX[d];
                int ny = currY + DY[d];
                if (isInside(nx, ny) && m_dist[nx][ny] == currentDist - 1) {
                    nextX = nx;
                    nextY = ny;
                    nextDir = static_cast<Direction>(d);
                    break;
                }
            }
        }

        if (nextX == -1) {
            break;
        }

        currX = nextX;
        currY = nextY;
        lastDir = nextDir;
    }

    if (currX != m_x || currY != m_y) {
        API::setColor(currX, currY, 'Y');
    } else {
        API::setColor(m_x, m_y, 'G');
    }

    log("Shortest path highlighted in YELLOW.");
}
