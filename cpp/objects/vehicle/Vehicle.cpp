#include "../vehicle/Vehicle.h"
#include "../sensors/Sensor.h"
#include "../../data/constants.h"
#include <spdlog/spdlog.h>
#include <limits>
#include <cmath>
#include <expected>

Vehicle::Vehicle(float length, float width,
                 std::array<uint8_t,3> color_bgr,
                 float rotation,
                 float init_speed,
                 float init_x,
                 float init_y,
                 float motor_force)
    : MapObject(length, width, b2_dynamicBody, rotation, init_speed, init_x, init_y,
        constants::vehicle_density, constants::vehicle_road_friction),
      color_bgr_(color_bgr)
{
    drive();
    max_motor_force_ = std::abs(mass() * constants::g);    // traction-limited
    max_brake_force_ = 4.0f * max_motor_force_;            // strong brakes
    pid_controller_.setSpeedLimits({-max_motor_force_, max_motor_force_, -50.0f, 50.0f});
    pid_controller_.setSteerLimits({-max_steering_angle_, max_steering_angle_, -0.2f, 0.2f});
    setMotorForce(motor_force); // initial motor force
}

Vehicle::~Vehicle() = default;

void Vehicle::addSensor(std::unique_ptr<Sensor> s, MountSide side, float offset)
{
    Sensor* sensor_ptr = s.get();
    sensors_.push_back(std::move(s)); // transfers ownership to vehicle
    if (map_) {
        setMap(map_); // re-register sensors on the map
    }
    // Provide owner/body/filter so the sensor can ignore us and reuse our collidable set
    sensor_ptr->setOwner(this);
    mounts_.push_back({sensor_ptr, side, offset}); // non-owning view used for pose updates
    updateSensors_();   // place immediately
}

void Vehicle::setMap(Map2D *map) {
    MapObject::setMap(map);
    for (auto& sensor : sensors_) {
        sensor->setMap(map);
    }
    updateSensors_(); // place sensors immediately
}

void Vehicle::updateSensors_() const {
    const float halfL = 0.5f * length();
    const float halfW = 0.5f * width();
    const b2BodyId id = body_descriptor_.bodyId;
    const b2Rot rot = rotation();
    const float theta = b2Rot_GetAngle(rot);

    for (auto& m : mounts_) {
        // Local (body-frame) mount offset
        b2Vec2 rel;
        switch (m.side) {
            case MountSide::Front:
                rel = { halfL + m.offset, 0.0f };
                break;
            case MountSide::Back:
                rel = { -halfL - m.offset, 0.0f };
                break;
            case MountSide::Left:
                rel = { 0.0f, halfW + m.offset};
                break;
            case MountSide::Right:
                rel = { 0.0f, -halfW - m.offset};
                break;
            default: // fallback to Front
                logger().warn("Unknown mount side. Defaulting to Front.");
                rel = { halfL + m.offset, 0.0f };
                break;
        }
        // Convert to world using Box2D helper
        const b2Vec2 worldP = b2Body_GetWorldPoint(id, rel);
        m.sensor->setPose(worldP, theta);
    }
}

void Vehicle::update() {
    // Calculate dt from this vehicle's last update time
    auto current_time = std::chrono::steady_clock::now();

    // Initialize last_update_time_ on first call
    if (last_updated_.time_since_epoch().count() == 0) {
        last_updated_ = current_time;
    }
    
    float dt = std::chrono::duration<float>(current_time - last_updated_).count();
    last_updated_ = current_time;
    
    // Use a minimum dt to avoid division by zero or very small values
    dt = std::max(dt, 1e-6f);
    while (true) {
        switch (mode_) {
            case Mode::Drive: {
                if (!follow_controller_) {
                    // Open-loop control: just apply current motor force and steering angle
                    setSteeringAngle(steering_angle_); // keep current steering angle
                    setMotorForce(motor_force_);       // keep current motor force
                    break;
                }
                if (!global_path_.empty()) {
                    // Perform local planning and control if a target path is set
                    auto local_path_res = findBestPath_();
                    if (!local_path_res.has_value()) {
                        logger().error("Local path planning failed: {}", local_path_res.error());
                        mode_ = Mode::Brake; // switch to brake mode
                        continue; // re-evaluate in Brake mode
                    }
                    const auto& local_path = local_path_res.value();
                    if (local_path.empty()) {
                        // No viable local path: stop the vehicle (fall through to braking style decel)
                        setSteeringAngle(0.0f);
                        setMotorForce(0.0f);
                    } else {
                        // Choose a point ahead to track (e.g., the first waypoint on the local path)
                        b2Vec2 target = local_path.size() > 1 ? local_path[1] : local_path[0];
                        // Compute control outputs
                        float current_speed = absoluteSpeed();
                        float current_angle = b2Rot_GetAngle(rotation());
                        auto [steer_ang, motor_force] = pid_controller_.compute(current_angle, position(), target, current_speed, desired_speed_, dt);
                        logger().debug("PID outputs: steer angle {:.3f} rad, motor force {:.1f} N", steer_ang, motor_force);
                        // Apply steering and throttle commands (with safety clamping)
                        setSteeringAngle(steer_ang);
                        setMotorForce(motor_force);
                    }
                    break;
                }
                // No path -> brake
                mode_ = Mode::Brake; // switch to brake mode
                logger().warn("No global path set. Switching to Brake mode.");
                continue;
            }
            case Mode::Rest: {
                setSteeringAngle(0.0f);
                setMotorForce(0.0f);
                break;
            }
            case Mode::Brake: {
                const float v_fwd = forwardSpeed();
                if (v_fwd < 0.01f) {
                    // Vehicle is nearly stopped: switch to Rest mode to avoid reversing
                    mode_ = Mode::Rest;
                    // zeroForwardVelocity_();
                    logger().debug("Vehicle speed is low: {:.2f} m/s. Switching to Rest mode.", v_fwd);
                    continue; // re-evaluate in Rest mode
                }
                setSteeringAngle(0.0f); // keep wheels straight during braking
                motor_force_ = brake_force_; // allows brake force to be much larger than max_motor_force_
                break;
            }
            default: {
                throw std::runtime_error("Unknown vehicle mode.");
            }
        }
        break;
    }
    notifyBox2DWorld_();
}

void Vehicle::updateFriction_() const {
    if (map_) {
        const b2BodyId id = body_descriptor_.bodyId;
        const b2Vec2 pos = b2Body_GetPosition(id);
        const auto [px, py] = map_->worldToCell(pos.x, pos.y);
        const Cell ground = map_->atPx(px, py);
        auto search = friction_map.find(ground);
        float linDamp = search != friction_map.end() ? search->second : 0.0f;
        b2Body_SetLinearDamping(id,  linDamp);
    }
}


void Vehicle::setMotorForce(float force)
{
    if (std::fabs(force) > max_motor_force_) {
        logger().warn("Requested motor force {:.1f} N exceeds max {:.1f} N. Clamping.",
                      force, max_motor_force_);
        motor_force_ = std::copysign(max_motor_force_, force);
    } else {
        motor_force_ = force;
    }
}

void Vehicle::brake(float force) {
    if (std::isnan(force)) {
        force = -motor_force_; // default: negative of current motor force
    }
    force = force < 0.0f ? force : -force; // ensure negative
    logger().info("Vehicle entering brake mode with force {:.1f} N.", force);
    if (force < -max_brake_force_) {
        logger().warn("Requested brake force {:.1f} N exceeds max {:.1f} N. Clamping.",
                      force, -max_brake_force_);
    }
    brake_force_ = std::clamp(force, -max_brake_force_, 0.0f);
    mode_ = Mode::Brake;
}


std::expected<std::vector<b2Vec2>, std::string> Vehicle::findBestPath_() {
    std::vector<b2Vec2> out;
    if (global_path_.empty()) return out;

    prunePassedWaypoints_();
    const auto [start,end] = computeHorizon_();
    global_path_[0] = start;

    // DEBUG
    std::cout << "Start position: (" << start.x << ", " << start.y << ")\n";
    std::cout << "End position: (" << end.x << ", " << end.y << ")\n";
    // END DEBUG

    // Build a fresh local graph config per lattice build to avoid mutating the global map graph
    GraphData local_graph_data_instance; // lives until function returns
    lattice_config_.graph_data = &local_graph_data_instance;

    std::vector<int> lastLayer;
    // const int startId = startNodePtr->getData();
    const int startId = buildLattice_(start, end, lastLayer);

    // choose target & shortest path
    const int tgt = chooseTargetNode_(startId, lastLayer);
    if (tgt < 0) {
        lattice_config_.graph_data = nullptr; // reset dangling ptr
        return out;
    }
    const auto sLbl = utils::nodeIdToLabel(startId);
    const auto tLbl = utils::nodeIdToLabel(tgt);

    auto *graph_data = lattice_config_.graph_data;
    auto itS = graph_data->graph_nodes.find(sLbl);
    auto itT = graph_data->graph_nodes.find(tLbl);
    if (itS == graph_data->graph_nodes.end() || !itS->second ||
        itT == graph_data->graph_nodes.end() || !itT->second) {
        lattice_config_.graph_data = nullptr; // reset dangling ptr
        return std::unexpected("Internal error: start/target node missing in graph.");
        }

    auto& startNode  = *itS->second;
    auto& targetNode = *itT->second;
    const auto res = graph_data->graph.dijkstra(startNode, targetNode);
    if (!res.success) {
        std::string ec = "No path could be found. Detail: " + res.errorMessage;
        lattice_config_.graph_data = nullptr; // reset dangling ptr
        return std::unexpected(ec);
    }
    auto pts = reconstructLocalPath_(res.path);
    lattice_config_.graph_data = nullptr; // reset dangling ptr
    return pts;
}

void Vehicle::notifyBox2DWorld_() const {
    const b2BodyId id = body_descriptor_.bodyId;
    // --- 1) Apply drive force at the steered FRONT axle (off-COM) ---
    const b2Vec2 frontLocal = {+0.5f * length(), 0.0f};
    const b2Vec2 wheelDirLocal = {std::cos(steering_angle_),
                                std::sin(steering_angle_)}; // +X rotated by steering

    // Convert to world
    const b2Vec2 frontWorld = b2Body_GetWorldPoint(id, frontLocal);
    const b2Vec2 wheelDirWorld  = b2Body_GetWorldVector(id, wheelDirLocal);

    // Drive force vector (traction-limited by setMotorForce / setAcceleration)
    const b2Vec2 Fdrive = { motor_force_ * wheelDirWorld.x,
                            motor_force_ * wheelDirWorld.y };
    b2Body_ApplyForce(id, Fdrive, frontWorld, true);
    updateFriction_();
    // "teleport" the sensors to the new vehicle pose
    updateSensors_();
}

std::pair<b2Vec2,b2Vec2> Vehicle::computeHorizon_() const {
    if (global_path_.empty()) return {position(), position()};
    const float horizont = lattice_config_.horizon_m;
    b2Vec2 start = position(),
    end = global_path_.back();
    float accumulated_distance=0.f;
    for (size_t i = 1; i < global_path_.size(); ++i){
        const float d = b2Distance(global_path_[i-1], global_path_[i]);
        if (accumulated_distance + d > horizont){
            const float t = (horizont-accumulated_distance)/std::max(d,1e-6f);
            end = { global_path_[i-1].x + t*(global_path_[i].x - global_path_[i-1].x),
                    global_path_[i-1].y + t*(global_path_[i].y - global_path_[i-1].y) };
            if (b2Distance(end, start) > lattice_config_.min_distance_between_nodes_) {
                break;
            }
        }
        accumulated_distance += d;
        end = global_path_[i];
        if (accumulated_distance>=horizont) break;
    }
    return {start,end};
}

int Vehicle::buildLattice_(const b2Vec2& start,
                            const b2Vec2& end,
                            std::vector<int>& lastLayer)
{
    auto *graph_data = lattice_config_.graph_data;
    auto *node_coords = &graph_data->node_coords;
    auto startNodePtr = utils::insertNewNodeReturnPtr(graph_data, start);
    const int startId = startNodePtr->getData();
    std::vector prevLayer{startId};

    auto [x, y] = end - start;
    const float L = std::hypot(x, y);
    if (L < 1e-6f){ lastLayer = prevLayer; return startId; }
    x/=L; y/=L;
    const b2Vec2 perp{-y, x};

    const unsigned int layers = std::max(1, static_cast<int>(std::floor(L / lattice_config_.step_m)));

    auto connectIfFree = [&](const int u, const int v){
        auto u_str = utils::nodeIdToLabel(u);
        auto v_str = utils::nodeIdToLabel(v);
        auto itA = node_coords->find(u_str);
        auto itB = node_coords->find(v_str);
        // nodes not found =>
        if (itA == node_coords->end() || itB == node_coords->end()) {
            return;
        }
        const b2Vec2& A = itA->second;
        const b2Vec2& B = itB->second;
        if (!edgeCollisionFree_(A,B)) return;
        auto node_1 = graph_data->graph_nodes[u_str];
        auto node_2 = graph_data->graph_nodes[v_str];

        auto new_edge = utils::insertDirectedEdgeReturnPtr(
            graph_data,
            node_1,
            node_2,
            b2Distance(A,B));
            };

    for (unsigned int i=1;i <= layers;++i){
        const float dist = (i<layers)? static_cast<float>(i) * lattice_config_.step_m : L;
        const b2Vec2 center{ start.x + dist*x, start.y + dist*y };

        std::vector<int> currLayer;
        currLayer.reserve(lattice_config_.nodes_per_step);
        const int mid = lattice_config_.nodes_per_step/2;
        for (int j=0;j<lattice_config_.nodes_per_step;++j){
            const float off = static_cast<float>(j - mid) * lattice_config_.lateral_spacing_m;
            const b2Vec2 p{ center.x + off*perp.x, center.y + off*perp.y };
            const int v = utils::insertNewNodeReturnID(graph_data, p);
            currLayer.push_back(v);
            for (const int u: prevLayer) {
                connectIfFree(u,v);
            }
        }
        prevLayer.swap(currLayer);
    }
    lastLayer = prevLayer;

    // NOTE: if you need the posMap later (for reconstruction), run Dijkstra now and reconstruct
    // inside this function; here we keep public API small and reconstruct via caller-provided map.
    return startId;
}

bool Vehicle::edgeCollisionFree_(const b2Vec2& a, const b2Vec2& b) const {
    unsigned int S = std::max(1u, lattice_config_.samples_along_edge_); // avoid division by zero
    const float halfL = 0.5f * length();
    const float halfW = 0.5f * width();
    for (unsigned int s = 0; s <= S; ++s){
        const float t = static_cast<float>(s) / static_cast<float>(S);
        const b2Vec2 c{ a.x + t*(b.x - a.x), a.y + t*(b.y - a.y) };
        const float ang = std::atan2(b.y-a.y, b.x-a.x);

        // OBB corners of vehicle at pose (center=c, heading=ang)
        const b2Vec2 d{std::cos(ang), std::sin(ang)};
        const b2Vec2 n{-d.y, d.x};
        const std::array obb = {
            c + halfL*d + halfW*n,
            c + halfL*d - halfW*n,
            c - halfL*d + halfW*n,
            c - halfL*d - halfW*n
        };

        // Fast raster check: reject if any corner hits obstacle cell.
        for (const auto& p: obb){
            auto [ix,iy] = map_->worldToCell(p.x, p.y);
            auto current_frame = map_->currentFrame();
            auto px = map_->atPx(ix, iy);
            if (px != Cell::Road && px != Cell::Free && px != Cell::Grass) return false;
            // TODO: check if any vehicle is located at the cell currently
        }
    }
    return true;
}

int Vehicle::chooseTargetNode_(
            unsigned int startNodeId,
            const std::vector<int>& lastLayer) const
{
    auto *graph_data = lattice_config_.graph_data;
    auto& posMap = graph_data->node_coords;
    auto startNodeCoord = posMap.find(utils::nodeIdToLabel(static_cast<int>(startNodeId)));
    if (startNodeCoord == posMap.end()) {
        throw std::runtime_error("Internal error: start node missing in graph.");
    }
    int best = -1;
    float md = std::numeric_limits<float>::infinity();
    for (int v: lastLayer){
        auto v_str = utils::nodeIdToLabel(v);
        const auto it = posMap.find(v_str);
        if (it == posMap.end()) continue;
        const float d = b2Distance(it->second,startNodeCoord->second);
        if (d < md) {
            md = d;
            best = v;
        }
    }
    return best;
}

std::vector<b2Vec2> Vehicle::reconstructLocalPath_(
    const std::vector<std::string>& path) const
{
    auto *graph_data = lattice_config_.graph_data;
    auto& posMap = graph_data->node_coords;
    std::vector<b2Vec2> pts;
    pts.reserve(path.size());
    for (const std::string& v: path){
        auto it = posMap.find(v);
        if (it!=posMap.end()) {
            pts.push_back(it->second);
        };
    }
    return pts;
}

void Vehicle::prunePassedWaypoints_()
{
    if (global_path_.size() < 2) return;

    const b2Vec2 P = position();
    const float reach_r = std::max(0.1f * length(), lattice_config_.min_distance_between_nodes_);

    // Keep pruning while the next waypoint is clearly behind us or already reached.
    while (global_path_.size() >= 2) {
        // Only P and a final B left -> drop B if we're basically there.
        if (global_path_.size() == 2) {
            const b2Vec2 B = global_path_[1];
            if (b2Distance(P, B) <= reach_r) {
                logger().debug("Prune: final waypoint reached (dist {:.2f} <= {:.2f}). Dropping B.",
                               b2Distance(P, B), reach_r);
                global_path_.erase(global_path_.begin() + 1);
                mode_ = Mode::Brake; // stop when we reach the end
                continue;
            }
            break; // nothing more to prune
        }

        const b2Vec2 B = global_path_[1];
        const b2Vec2 C = global_path_[2];
        const b2Vec2 BC{ C.x - B.x, C.y - B.y };
        const float L2 = BC.x * BC.x + BC.y * BC.y;

        // Degenerate next segment -> drop B
        if (L2 < 1e-6f) {
            logger().debug("Prune: degenerate segment |C-B|≈0. Dropping B... Current position P=({:.2f},{:.2f}), B=({:.2f},{:.2f}), C=({:.2f},{:.2f})",
                       P.x, P.y, B.x, B.y, C.x, C.y);
            global_path_.erase(global_path_.begin() + 1);
            continue;
        }

        const b2Vec2 BP{ P.x - B.x, P.y - B.y };
        const float invL   = 1.0f / std::sqrt(L2);
        const float t      = (BP.x * BC.x + BP.y * BC.y) / L2;      // normalized along-track
        const float s      = (BP.x * BC.x + BP.y * BC.y) * invL;    // metric along-track
        const float crossZ = std::fabs(BP.x * BC.y - BP.y * BC.x);  // 2D "cross"
        const float e_xt   = crossZ * invL;                          // cross-track error

        // Heuristics:
        // - t > 0 → we are ahead of B along the direction towards C (i.e., we passed B).
        // - e_xt small → we are roughly on the corridor around the path.
        // - Or, if we are simply close to B, also drop it.
        const bool ahead_of_B   = (t > 0.0f + 1e-3f);
        const bool near_B       = (b2Distance(P, B) <= reach_r);
        const bool corridor_ok  = (e_xt <= 2.0f * width()) || (s > 0.5f * lattice_config_.min_distance_between_nodes_);

        if (ahead_of_B && corridor_ok && e_xt < lattice_config_.min_distance_between_nodes_) {
            logger().debug("Prune: waypoint[1] behind vehicle (t={:.2f}, e_xt={:.2f}m, s={:.2f}m). Dropping B... "
                           "Current position: P=({:.2f},{:.2f}), B=({:.2f},{:.2f}), C=({:.2f},{:.2f})",
                           t, e_xt, s, P.x, P.y, B.x, B.y, C.x, C.y);
            global_path_.erase(global_path_.begin() + 1);
            continue;
        }
        if (near_B) {
            logger().debug("Prune: near waypoint[1] (dist {:.2f} <= {:.2f}). Dropping B... current position: P=({:.2f},{:.2f}), B=({:.2f},{:.2f}), C=({:.2f},{:.2f})",
                           b2Distance(P, B), reach_r, P.x, P.y, B.x, B.y, C.x, C.y);
            global_path_.erase(global_path_.begin() + 1);
            continue;
        }
        break; // next waypoint is OK to keep
    }
}