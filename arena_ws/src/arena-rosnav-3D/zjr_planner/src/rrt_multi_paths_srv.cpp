// rrt_multi_paths_srv.cpp
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <zjr_planner/GenerateRRTPaths.h>
#include <vector>
#include <random>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <filesystem>

using namespace std;

struct Node {
    double x;
    double y;
    Node* parent;
    Node(double _x=0, double _y=0, Node* _p=nullptr) : x(_x), y(_y), parent(_p) {}
};

class GridMap {
public:
    int width, height;
    double resolution;
    double origin_x, origin_y;
    std::vector<int8_t> data; // row-major: index = y*width + x

    GridMap() = default;
    GridMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        width = msg->info.width;
        height = msg->info.height;
        resolution = msg->info.resolution;
        origin_x = msg->info.origin.position.x;
        origin_y = msg->info.origin.position.y;
        data = msg->data;
    }

    pair<int,int> worldToMap(double xw, double yw) const {
        int mx = int((xw - origin_x) / resolution);
        int my = int((yw - origin_y) / resolution);
        return {mx, my};
    }

    pair<double,double> mapToWorld(int mx, int my) const {
        double xw = origin_x + (mx + 0.5) * resolution;
        double yw = origin_y + (my + 0.5) * resolution;
        return {xw, yw};
    }

    bool inBoundsMap(int mx, int my) const {
        return (mx >= 0 && mx < width && my >= 0 && my < height);
    }

    bool isOccupiedMap(int mx, int my) const {
        if (!inBoundsMap(mx,my)) return true;
        int idx = my * width + mx;
        int val = static_cast<int>(data[idx]);
        return val > 50;
    }

    bool isFreeWorld(double xw, double yw) const {
        auto [mx,my] = worldToMap(xw,yw);
        return !isOccupiedMap(mx,my);
    }

    // Bresenham-like discrete line checking on map indices
    bool lineCollisionCheck(double x1, double y1, double x2, double y2) const {
        auto [mx1,my1] = worldToMap(x1,y1);
        auto [mx2,my2] = worldToMap(x2,y2);
        if (!inBoundsMap(mx1,my1) || !inBoundsMap(mx2,my2)) return true;
        int dx = abs(mx2 - mx1);
        int dy = abs(my2 - my1);
        int x = mx1;
        int y = my1;
        int sx = (mx2 >= mx1) ? 1 : -1;
        int sy = (my2 >= my1) ? 1 : -1;
        if (dx >= dy) {
            int err = dx / 2;
            while (x != mx2) {
                if (isOccupiedMap(x,y)) return true;
                err -= dy;
                if (err < 0) { y += sy; err += dx; }
                x += sx;
            }
            if (isOccupiedMap(mx2,my2)) return true;
        } else {
            int err = dy / 2;
            while (y != my2) {
                if (isOccupiedMap(x,y)) return true;
                err -= dx;
                if (err < 0) { x += sx; err += dy; }
                y += sy;
            }
            if (isOccupiedMap(mx2,my2)) return true;
        }
        return false;
    }
};

double computePathLength(const vector<pair<double,double>>& path) {
    if (path.size() < 2) return 0.0;
    double total = 0.0;
    for (size_t i=1;i<path.size();++i) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        total += hypot(dx,dy);
    }
    return round(total * 100.0) / 100.0; // two decimals
}

class RRTPlanner {
public:
    GridMap grid;
    Node start;
    Node goal;
    double step_size;
    int max_iters;
    double goal_sample_rate;
    double min_x, min_y, max_x, max_y;

    std::mt19937 rng;
    std::uniform_real_distribution<double> uni_x;
    std::uniform_real_distribution<double> uni_y;
    std::uniform_real_distribution<double> uni01;

    RRTPlanner(const GridMap& g, pair<double,double> s, pair<double,double> gl,
               double step=0.5, int maxit=5000, double gsr=0.05, unsigned seed=0)
        : grid(g), start(s.first,s.second,nullptr), goal(gl.first,gl.second,nullptr),
          step_size(step), max_iters(maxit), goal_sample_rate(gsr),
          rng(seed),
          uni_x(g.origin_x, g.origin_x + g.width * g.resolution),
          uni_y(g.origin_y, g.origin_y + g.height * g.resolution),
          uni01(0.0,1.0)
    {
        min_x = g.origin_x;
        min_y = g.origin_y;
        max_x = g.origin_x + g.width * g.resolution;
        max_y = g.origin_y + g.height * g.resolution;
    }

    pair<double,double> sample() {
        double p = uni01(rng);
        if (p < goal_sample_rate) return {goal.x, goal.y};
        double x = uni_x(rng);
        double y = uni_y(rng);
        return {x,y};
    }

    Node* nearest(const vector<Node*>& nodes, double x, double y) {
        Node* best = nullptr;
        double best_d = std::numeric_limits<double>::infinity();
        for (Node* n: nodes) {
            double d = (n->x - x)*(n->x - x) + (n->y - y)*(n->y - y);
            if (d < best_d) { best_d = d; best = n; }
        }
        return best;
    }

    pair<double,double> steer(Node* from, double tx, double ty) {
        double dx = tx - from->x;
        double dy = ty - from->y;
        double dist = hypot(dx,dy);
        if (dist <= step_size) return {tx, ty};
        double nx = from->x + dx / dist * step_size;
        double ny = from->y + dy / dist * step_size;
        return {nx, ny};
    }

    vector<pair<double,double>> extractPath(Node* node) {
        vector<pair<double,double>> path;
        Node* cur = node;
        while (cur) {
            path.emplace_back(cur->x, cur->y);
            cur = cur->parent;
        }
        reverse(path.begin(), path.end());
        return path;
    }

    vector<pair<double,double>> smoothPath(const vector<pair<double,double>>& in_path, int iterations=50) {
        if (in_path.size() < 3) return in_path;
        vector<pair<double,double>> path = in_path;
        std::uniform_int_distribution<int> idist;
        for (int it=0; it<iterations; ++it) {
            if (path.size() < 3) break;
            int n = (int)path.size();
            std::uniform_int_distribution<int> i_dist(0, n-3);
            int i = i_dist(rng);
            std::uniform_int_distribution<int> j_dist(i+2, n-1);
            int j = j_dist(rng);
            auto [x1,y1] = path[i];
            auto [x2,y2] = path[j];
            if (!grid.lineCollisionCheck(x1,y1,x2,y2)) {
                vector<pair<double,double>> newp;
                newp.insert(newp.end(), path.begin(), path.begin()+i+1);
                newp.insert(newp.end(), path.begin()+j, path.end());
                path.swap(newp);
            }
        }
        return path;
    }

    // returns empty vector if failed
    vector<pair<double,double>> build() {
        vector<Node*> nodes;
        nodes.push_back(new Node(start.x, start.y, nullptr));
        for (int iter=0; iter<max_iters; ++iter) {
            auto [rx,ry] = sample();
            Node* nearest_node = nearest(nodes, rx, ry);
            auto [nx,ny] = steer(nearest_node, rx, ry);
            if (grid.lineCollisionCheck(nearest_node->x, nearest_node->y, nx, ny)) {
                continue;
            }
            Node* newnode = new Node(nx, ny, nearest_node);
            nodes.push_back(newnode);
            double dist_to_goal = hypot(newnode->x - goal.x, newnode->y - goal.y);
            if (dist_to_goal <= step_size) {
                if (!grid.lineCollisionCheck(newnode->x, newnode->y, goal.x, goal.y)) {
                    Node* goalnode = new Node(goal.x, goal.y, newnode);
                    auto path = extractPath(goalnode);
                    // clean allocated nodes
                    for (Node* p : nodes) delete p;
                    delete goalnode;
                    return path;
                }
            }
        }
        for (Node* p : nodes) delete p;
        return {};
    }
};

// helper to format double with 3 decimals as string
string fmt3(double v) {
    std::ostringstream oss;
    oss<<fixed<<setprecision(3)<<v;
    return oss.str();
}

bool handle_generate(zjr_planner::GenerateRRTPaths::Request &req,
                     zjr_planner::GenerateRRTPaths::Response &res)
{
    // 1) wait for map and odom (like Python wait_for_message)
    auto map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(15.0));
    if (!map_msg) {
        ROS_ERROR("Timed out waiting for /map");
        res.success = false;
        res.filename = "";
        res.json = "";
        return true;
    }
    GridMap grid(map_msg);

    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(5.0));
    if (!odom_msg) {
        ROS_ERROR("Timed out waiting for /odom");
        res.success = false;
        res.filename = "";
        res.json = "";
        return true;
    }
    double sx = odom_msg->pose.pose.position.x;
    double sy = odom_msg->pose.pose.position.y;
    ROS_INFO("Start from /odom: (%.3f, %.3f)", sx, sy);

    double gx = req.goal_x;
    double gy = req.goal_y;
    int num_paths = (req.num_paths > 0) ? req.num_paths : 3;
    double step = (req.step > 0.0) ? req.step : 0.5;
    int max_iters = (req.max_iters > 0) ? req.max_iters : 5000;

    // check free space
    if (!grid.isFreeWorld(sx, sy)) {
        ROS_ERROR("Start is in occupied cell. Abort.");
        res.success = false;
        return true;
    }
    if (!grid.isFreeWorld(gx, gy)) {
        ROS_ERROR("Goal is in occupied cell. Abort.");
        res.success = false;
        return true;
    }

    // prepare multiple attempts
    vector<vector<pair<double,double>>> final_paths; // store successful paths in order
    vector<double> final_lengths;
    int generated = 0;
    int attempts = 0;
    int max_attempts = num_paths * 10;
    unsigned base_seed = static_cast<unsigned>(time(nullptr)) & 0xffffffff;

    while (generated < num_paths && attempts < max_attempts) {
        ++attempts;
        unsigned seed = base_seed + attempts;
        ROS_INFO("RRT attempt %d seed=%u", attempts, seed);
        RRTPlanner planner(grid, {sx,sy}, {gx,gy}, step, max_iters, 0.1, seed);
        auto raw = planner.build();
        if (raw.empty()) {
            ROS_WARN("RRT failed to find path in this attempt.");
            continue;
        }
        auto smooth = planner.smoothPath(raw, 10);
        // collision check for segment-wise path
        bool collision = false;
        for (size_t i=1;i<smooth.size();++i) {
            if (grid.lineCollisionCheck(smooth[i-1].first, smooth[i-1].second,
                                        smooth[i].first, smooth[i].second)) {
                collision = true;
                break;
            }
        }
        if (collision) {
            ROS_WARN("Path has collision after smoothing, skip.");
            continue;
        }
        // uniqueness check against existing in final_paths
        bool too_similar = false;
        for (const auto &prev : final_paths) {
            size_t L = std::min(prev.size(), smooth.size());
            if (L >= 2) {
                double dsum = 0.0;
                for (size_t i=0;i<L;++i) {
                    double dx = prev[i].first - smooth[i].first;
                    double dy = prev[i].second - smooth[i].second;
                    dsum += hypot(dx,dy);
                }
                double avg = dsum / static_cast<double>(L);
                if (avg < 0.2) { too_similar = true; break; }
            }
        }
        if (too_similar) {
            ROS_INFO("Generated path too similar to previous, retrying.");
            continue;
        }
        // accept path
        ++generated;
        double length = computePathLength(smooth);
        final_paths.push_back(smooth);
        final_lengths.push_back(length);
        ROS_INFO("Generated path %d : %lu points length %.2f", generated, smooth.size(), length);
    }

    // If no path generated at all, produce placeholder direct start->goal path
    if (final_paths.empty()) {
        ROS_WARN("No valid RRT path found. Creating placeholder straight-line path.");
        vector<pair<double,double>> placeholder = {{sx, sy}, {gx, gy}};
        final_paths.push_back(placeholder);
        final_lengths.push_back(computePathLength(placeholder));
    }

    // If we generated fewer than requested, repeat last path to fill to num_paths.
    while ((int)final_paths.size() < num_paths) {
        ROS_WARN("Generated paths (%zu) < num_paths (%d). Repeating last path to fill.", final_paths.size(), num_paths);
        final_paths.push_back(final_paths.back());
        final_lengths.push_back(final_lengths.back());
    }

    // Build JSON string in deterministic order path_1_1_1 ... path_1_1_N
    std::ostringstream jsonoss;
    jsonoss << "{\n";
    for (int i=0; i<num_paths; ++i) {
        const auto &path = final_paths[i];
        double length = final_lengths[i];
        jsonoss << "  \"path_1_1_" << (i+1) << "\": {\n";
        jsonoss << "    \"path\": [\n";
        for (size_t j=0;j<path.size();++j) {
            jsonoss << "      {\"position\": [" << fmt3(path[j].first) << ", " << fmt3(path[j].second) << "]}";
            if (j+1<path.size()) jsonoss << ",";
            jsonoss << "\n";
        }
        jsonoss << "    ],\n";
        jsonoss << "    \"length\": " << std::fixed << std::setprecision(2) << length << "\n";
        jsonoss << "  }";
        if (i+1<num_paths) jsonoss << ",";
        jsonoss << "\n";
    }
    jsonoss << "}\n";
    string json_str = jsonoss.str();

    // Save to file (default path similar to your python)
    std::filesystem::path save_dir = std::filesystem::path(std::getenv("HOME")) / "catkin_arena" / "src" / "zjr_planner" / "scripts" / "data_json";
    try {
        std::filesystem::create_directories(save_dir);
    } catch (...) {
        ROS_WARN("Could not create directory, will try to save in /tmp");
        save_dir = "/tmp";
    }
    std::time_t t = std::time(nullptr);
    std::ostringstream fn;
    fn << "paths.json";
    std::filesystem::path filename = save_dir / fn.str();

    std::ofstream ofs(filename.string());
    if (!ofs) {
        ROS_ERROR("Failed to open file %s for writing", filename.string().c_str());
        res.success = false;
        return true;
    }
    ofs << json_str;
    ofs.close();

    res.success = true;
    res.filename = filename.string();
    res.json = json_str;
    ROS_INFO("Saved %d paths to %s", num_paths, res.filename.c_str());
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_multi_paths_srv");
    ros::NodeHandle nh;

    ros::ServiceServer srv = nh.advertiseService("/generate_rrt_paths", handle_generate);
    ROS_INFO("Service /generate_rrt_paths ready. Call it with goal_x, goal_y, num_paths, step, max_iters.");

    ros::spin();
    return 0;
}
