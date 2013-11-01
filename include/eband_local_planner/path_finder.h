#ifndef EBAND_PATH_FINDER 
#define EBAND_PATH_FINDER

namespace eband_local_planner {

  void reconstructPath(const std::vector<int> &came_from, int current_id, 
      std::vector<std::pair<int,int> > &path, int size_x) {
    std::pair<int, int> current(current_id % size_x, current_id / size_x);
    if (came_from[current_id] != -1) {
      reconstructPath(came_from, came_from[current_id], path, size_x);
      path.push_back(current);
    } else {
      path.clear();
      path.push_back(current);
    }
  }

  int getBestVertex(const std::set<int> &vertices, const std::vector<float> &scores) {
    float min_score = std::numeric_limits<float>::max();
    int min_vertex_id = -1;
    for (std::set<int>::const_iterator it = vertices.begin(); it != vertices.end(); it++) {
      float vertex_score = scores[*it];
      if (vertex_score < min_score) {
        min_score = vertex_score;
        min_vertex_id = *it;
      }
    }
    return min_vertex_id;
  }

  bool searchForPath(unsigned char * map, int size_x, int size_y, 
      int start_x, int start_y, int goal_x, int goal_y, 
      std::vector<std::pair<int, int> > &path) {

    int start = size_x * start_y + start_x;
    int goal = size_x * goal_y + goal_x;
    std::set<int> open_set;
    open_set.insert(start);

    std::vector<bool> closed_set(size_x * size_y, false);
    std::vector<int> came_from(size_x * size_y, -1);
    std::vector<float> g_score(size_x * size_y, 0.0f);
    std::vector<float> h_score(size_x * size_y, 0.0f);
    std::vector<float> f_score(size_x * size_y, 0.0f);

    g_score[start] = 0.0f;
    h_score[start] = sqrt((start_x-goal_x)*(start_x-goal_x) +
        (start_y-goal_y)*(start_y-goal_y));
    f_score[start] = g_score[start] + h_score[start];

    while (!open_set.empty()) {
      int current = getBestVertex(open_set, f_score);
      if (current == goal) {
        path.clear();
        if (came_from[goal] != -1) {
          reconstructPath(came_from, came_from[goal], path, size_x);
        }
        return true;
      }

      open_set.erase(current);
      closed_set[current] = true;

      int current_x = current % size_x;
      int current_y = current / size_x;

      int x_offset[] = {-1, 0, 1, -1, 1, -1, 0, 1};
      int y_offset[] = {-1, -1, -1, 0, 0, 1, 1, 1};
      for (int n_num = 0; n_num < 8; ++n_num) {
        int n_x = current_x + x_offset[n_num];
        int n_y = current_y + y_offset[n_num];
        if (n_x < 0 || n_x >= size_x || n_y < 0 || n_y > size_y) {
          continue;
        }
        int n = n_y * size_x + n_x;
        if (map[n] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
            closed_set[n]) {
          continue;
        }

        float tentative_g_score = g_score[current] + 
            sqrt((n_x-current_x)*(n_x-current_x)+(n_y-current_y)*(n_y-current_y)) +
            log(map[n] + 1)/10.0f;

        bool tentative_is_better = false;
        if (!open_set.count(n)) {
          open_set.insert(n);
          tentative_is_better = true;
        } else if (tentative_g_score < g_score[n]) {
          tentative_is_better = true;
        }

        if (tentative_is_better) {
          came_from[n] = current;
          g_score[n] = tentative_g_score;
          h_score[n] = sqrt((n_x-goal_x)*(n_x-goal_x)+(n_y-goal_y)*(n_y-goal_y));
          f_score[n] = g_score[n] + h_score[n];
        }
      }
    }

    return false;
  }

}

#endif
