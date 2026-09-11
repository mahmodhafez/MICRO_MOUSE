#pragma once

#include <string>

// Absolute cardinal directions in the maze
enum Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

class FloodFill {
public:
    FloodFill();

    // Initialize maze state, perimeter walls, start position, and initial distances
    void init();

    // Execute a single step: sense, turn, move, and update visualization
    // Returns true if mouse moved, false if reached goal or stopped
    bool step();

    // Check if the mouse is currently at one of the center goal cells
    bool isAtGoal() const;

    // Reconstruct and highlight the shortest path from start (0, 0) to goal in Yellow
    void highlightShortestPath();

    // Dynamically seal physically verified dead ends behind the mouse
    void checkAndSealDeadEnd(int x, int y);

    // Getters for current position and heading
    int getX() const { return m_x; }
    int getY() const { return m_y; }
    Direction getHeading() const { return m_heading; }

private:
    static const int MAZE_WIDTH = 16;
    static const int MAZE_HEIGHT = 16;

    int m_x;
    int m_y;
    Direction m_heading;

    // m_walls[x][y][dir] is true ONLY when physically sensed or boundary
    bool m_walls[MAZE_WIDTH][MAZE_HEIGHT][4];

    // Distance grid representing shortest open path length to goal cells
    int m_dist[MAZE_WIDTH][MAZE_HEIGHT];

    // Cache of displayed distances on MMS to avoid redundant setText commands
    int m_displayedDist[MAZE_WIDTH][MAZE_HEIGHT];

    // Visited cells tracker
    bool m_visited[MAZE_WIDTH][MAZE_HEIGHT];

    // Helpers
    void log(const std::string& message) const;
    char directionToChar(Direction dir) const;
    bool isInside(int x, int y) const;
    bool isGoalCell(int x, int y) const;

    // Wall sensing and updating (strictly physical walls)
    bool senseAndUpdateWalls();
    void addWall(int x, int y, Direction dir);

    // BFS Flood Fill algorithm
    void recalculateDistances();
    void updateDisplayedTexts();

    // Turn-penalized navigation
    Direction getBestNextDirection();
    void turnTo(Direction targetHeading);
    void moveOneCell();
};
