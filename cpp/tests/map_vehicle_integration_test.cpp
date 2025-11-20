#include <gtest/gtest.h>
#include "../map/Map2D.h"
#include "../objects/vehicle/Vehicle.h"
#include "../objects/Road.h"
#include "../setup/Setup.h"
#include <memory>
#include <vector>
#include <filesystem>

static std::shared_ptr<spdlog::logger> testLogger = spdlog::stdout_color_mt("MapVehicleIntegrationTest");
static std::filesystem::path output_debug_path = "/home/critter/workspace/autonomous_vehicle_simulator/cpp/tests/debug";

/**
 * @test Integration test that combines Map2D graph construction with Vehicle path following.
 *
 * This test:
 * 1. Loads a dummy map from "dummy.png"
 * 2. Constructs a road graph using Map2D's graph construction methods
 * 3. Extracts a path from the constructed graph
 * 4. Sets the vehicle's global path to this path
 * 5. Runs the vehicle automatically to follow the path
 */
TEST(MapVehicleIntegration, GraphConstructionAndPathFollowing)
{
    setupWorld();

    // Create a map with some road objects to form a path
    Map2D map(500, 500);

    testLogger->info("Creating road network for graph construction");

    // Create a simple road network: a straight path with a turn
    // Horizontal road segment
    for (int i = 0; i < 5; i++) {
        auto road = std::make_unique<Road>(10.0f, 10.0f, 10.0f + 10.0f * i, 10.0f);
        map.addObject(std::move(road));
    }

    // Vertical road segment (creates an intersection)
    for (int i = 0; i < 5; i++) {
        auto road = std::make_unique<Road>(10.0f, 10.0f, 30.0f, 10.0f + 10.0f * i, M_PI / 2.0f);
        map.addObject(std::move(road));
    }

    testLogger->info("Building graph from road network");

    // Build the graph from the road network
    map.buildGraph();

    auto& graph_data = map.graphData();

    // Verify that the graph was constructed
    ASSERT_GT(graph_data.graph.getNodeSet().size(), 0) << "Graph should contain nodes";
    ASSERT_GT(graph_data.graph.getEdgeSet().size(), 0) << "Graph should contain edges";

    testLogger->info("Graph constructed with {} nodes and {} edges",
                    graph_data.graph.getNodeSet().size(),
                    graph_data.graph.getEdgeSet().size());

    // Extract a path from the graph (use the node coordinates)
    std::vector<b2Vec2> path;

    // Sort path by x-coordinate to create a reasonable traversal order
    std::ranges::sort(path, [](const b2Vec2& a, const b2Vec2& b) {
        return a.x < b.x;
    });

    for (const auto& [label, coord] : graph_data.node_coords) {
        path.push_back(coord);
        testLogger->info("Node {}: ({:.2f}, {:.2f})", label, coord.x, coord.y);
    }

    ASSERT_GE(path.size(), 2) << "Path must contain at least 2 waypoints";

    testLogger->info("Extracted path with {} waypoints", path.size());

    // Create a vehicle near the start of the path
    auto vehicle = std::make_unique<Vehicle>(
        4.0f,  // length
        2.0f,  // width
        std::array<uint8_t, 3>{255, 0, 255},  // color (magenta)
        0.0f,  // rotation
        0.0f,  // initial speed
        path[0].x,  // start x position
        path[0].y,  // start y position
        13000.0f  // motor force
    );

    Vehicle* vehicle_ptr = vehicle.get();
    map.addObject(std::move(vehicle));

    // Set the global path from the graph
    testLogger->info("Setting vehicle's global path from constructed graph");
    vehicle_ptr->setGlobalPath(path);

    // Enable the controller for automatic path following
    vehicle_ptr->enableController();
    vehicle_ptr->setDesiredAbsoulteSpeed(35.0f);
    vehicle_ptr->startUpdating();

    testLogger->info("Starting simulation");
    map.startSimulation();

    // Run simulation for a reasonable duration
    auto start_time = std::chrono::steady_clock::now();
    auto simulation_duration = std::chrono::seconds(10);

    while (std::chrono::steady_clock::now() - start_time < simulation_duration) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto pos = vehicle_ptr->position();
        auto speed = vehicle_ptr->absoluteSpeed();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time
        ).count();

        testLogger->info("Time: {}s, Position: ({:.2f}, {:.2f}), Speed: {:.2f} m/s",
                        elapsed, pos.x, pos.y, speed);
    }

    map.endSimulation();

    // Save the simulation video
    std::filesystem::path videoFile = output_debug_path / "map_vehicle_integration_test.mp4";
    map.flushFrames(videoFile.string());

    testLogger->info("Simulation video saved to: {}", videoFile.string());

    // Verify that the video was created
    ASSERT_TRUE(std::filesystem::exists(videoFile)) << "Video file should be created";
    EXPECT_GT(std::filesystem::file_size(videoFile), 0) << "Video file should not be empty";

    // Verify that the vehicle moved from its starting position
    auto final_pos = vehicle_ptr->position();
    auto distance_traveled = std::sqrt(
        std::pow(final_pos.x - path[0].x, 2) +
        std::pow(final_pos.y - path[0].y, 2)
    );

    testLogger->info("Distance traveled: {:.2f} meters", distance_traveled);
    EXPECT_GT(distance_traveled, 1.0f) << "Vehicle should have moved at least 1 meter";

    destroyWorld();
}

/**
 * @test Test loading a Test map file and constructing a graph from it.
 *
 * This test attempts to load "map_unittest_automatic_graph_construction.png" from the maps directory and construct
 * a graph from the road objects in the map.
 */
TEST(MapVehicleIntegration, LoadDummyMapAndConstructGraph)
{
    setupWorld();

    // Path to dummy map (adjust as needed)
    std::string dummy_map_file = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/map_unittest_automatic_graph_construction.bmp";
    std::string dummy_metadata_file = "/home/critter/workspace/autonomous_vehicle_simulator/res/maps/map_unittest_automatic_graph_construction.json";

    testLogger->info("Attempting to load dummy map from: {}", dummy_map_file);

    // Check if the files exist before attempting to load
    if (!std::filesystem::exists(dummy_map_file)) {
        testLogger->warn("Dummy map file not found at: {}. Skipping test.", dummy_map_file);
        GTEST_SKIP() << "Dummy map file not found: " << dummy_map_file;
    }

    if (!std::filesystem::exists(dummy_metadata_file)) {
        testLogger->warn("Dummy metadata file not found at: {}. Skipping test.", dummy_metadata_file);
        GTEST_SKIP() << "Dummy metadata file not found: " << dummy_metadata_file;
    }

    // Load the map
    Map2D map = Map2D::loadMap(dummy_map_file, dummy_metadata_file);

    testLogger->info("Loaded dummy map: {}x{} pixels", map.width(), map.height());

    // Build the graph
    testLogger->info("Building graph from dummy map");
    map.buildGraph();

    auto& graph_data = map.graphData();

    testLogger->info("Graph constructed with {} nodes and {} edges",
                    graph_data.graph.getNodeSet().size(),
                    graph_data.graph.getEdgeSet().size());

    // If the map has road objects, the graph should have nodes
    EXPECT_GT(graph_data.graph.getNodeSet().size(), 0) << "Graph should contain nodes if map has roads";

    // Extract path from graph
    if (graph_data.node_coords.size() >= 2) {
        std::vector<b2Vec2> path;
        for (const auto& [label, coord] : graph_data.node_coords) {
            path.push_back(coord);
        }

        // Create a vehicle and set the path
        auto vehicle = std::make_unique<Vehicle>(
            4.5f, 2.0f,
            std::array<uint8_t, 3>{255, 0, 255},
            0.0f, 0.0f,
            path[0].x, path[0].y,
            8000.0f
        );

        Vehicle* vehicle_ptr = vehicle.get();
        map.addObject(std::move(vehicle));

        vehicle_ptr->setGlobalPath(path);
        vehicle_ptr->enableController();
        vehicle_ptr->setDesiredAbsoulteSpeed(20.0f);
        vehicle_ptr->startUpdating();

        // Run simulation
        map.startSimulation();
        std::this_thread::sleep_for(std::chrono::seconds(8));
        map.endSimulation();

        // Save video
        std::filesystem::path videoFile = output_debug_path / "map_unittest_automatic_graph_construction_vehicle_test.mp4";
        map.flushFrames(videoFile.string());

        testLogger->info("Simulation video saved to: {}", videoFile.string());

        ASSERT_TRUE(std::filesystem::exists(videoFile));
        EXPECT_GT(std::filesystem::file_size(videoFile), 0);
    } else {
        testLogger->warn("Not enough nodes in graph to create a path");
    }

    destroyWorld();
}
