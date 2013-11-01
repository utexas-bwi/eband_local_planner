#ifndef EBAND_DIRECTED_DFS 
#define EBAND_DIRECTED_DFS

namespace eband_local_planner {


namespace topological_mapper {

  /**
   * \brief Non-recusrive start point for performing DFS
   */
  bool searchForPath(unsigned char * map, unsigned nx, unsigned ny, 
      unsigned start_x, unsigned start_y, unsigned goal_x, unsigned goal_y, 
      std::vector<std::pair<unsigned, unsigned> > &path) {
    path.clear();
    std::vector<bool> visited(nx * ny, false);
    return searchForPath(map, nx, ny, start_x, start_y, goal_x, goal_y, path, visited);
  }

  /**
   * \brief Recusrive function performing DFS
   */
  bool searchForPath(unsigned char * map, unsigned nx, unsigned ny, 
      unsigned start_x, unsigned start_y, unsigned goal_x, unsigned goal_y, 
      std::vector<std::pair<unsigned, unsigned> > &path, std::vector<bool> &visited) {

    // Termination crit
    if (start_x == goal_x && start_y == goal_y) {
      return true;
    }

    uint32_t start_idx = nx * start_y + start_X;
    visited[start_idx] = true;

    std::vector<Point2d> neighbours;
    getOrderedNeighbours(start, goal, visited, neighbours, in_obstacle_space);
    for (size_t i = 0; i < neighbours.size(); ++i) {
      Point2d& n = neighbours[i];
      // Check if it has been visited again - quite likely that one of the
      // previous loop iterations have covered this already
      uint32_t n_idx = MAP_IDX(map_.info.width, n.x, n.y);
      if (visited[n_idx]) {
        continue;
      }
      bool success = searchForPath(n, goal, depth - 1, visited);
      if (success)
        return true;
    }

    return false; // disconnected components
  }


  /**
   * \brief Gets neighbours for a given node iff they are also obstacles
   * and have not been visited before
   */
  void DirectedDFS::getOrderedNeighbours(const Point2d &from,
      const Point2d &goal, const std::vector<bool> &visited,
      std::vector<Point2d> &neighbours, bool in_obstacle_space) {

    size_t neighbour_count = 8;
    uint32_t x_offset[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    uint32_t y_offset[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    neighbours.clear();
    for (size_t i = 0; i < neighbour_count; ++i) {
      Point2d p = from + Point2d(x_offset[i],y_offset[i]);
      if (p.x >= (int) map_.info.width || p.y >= (int) map_.info.height ||
          p.x <= 0 || p.y <= 0) {
        continue;
      }
      uint32_t map_idx = MAP_IDX(map_.info.width, p.x, p.y);
      if (visited[map_idx] || (in_obstacle_space && map_.data[map_idx] == 0) ||
          (!in_obstacle_space && map_.data[map_idx] != 0)) {
        continue;
      }
      p.distance_from_ref = norm(p - goal);
      neighbours.push_back(p);
    }
    std::sort(neighbours.begin(), neighbours.end(), Point2dDistanceComp());
  }

}

#endif
