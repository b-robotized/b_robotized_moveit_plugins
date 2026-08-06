#include <gtest/gtest.h>
#include "cubic_spline_smoothing_adapter/cubic_spline_smoothing.hpp"
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

using namespace cubic_spline_smoothing_adapter;

class SplineSmoothingTest : public ::testing::Test {
protected:
    KinematicLimits limits;
    SmoothingConfig config;
    moveit::core::RobotModelPtr robot_model;
    const moveit::core::JointModelGroup* jmg;

    void SetUp() override {
        limits.v_max = {3.0, 3.0};
        limits.a_max = {5.0, 5.0};
        limits.a_min = {-5.0, -5.0};
        limits.p_min = {-3.14, -3.14};
        limits.p_max = {3.14, 3.14};
        config.max_time_stretch = 10.0;
        config.path_dev_max = 0.20;

        const std::string urdf_xml =
            "<?xml version=\"1.0\" ?>"
            "<robot name=\"test_robot\">"
            "  <link name=\"base_link\"/>"
            "  <link name=\"link1\"/>"
            "  <link name=\"link2\"/>"
            "  <joint name=\"joint1\" type=\"revolute\">"
            "    <parent link=\"base_link\"/><child link=\"link1\"/>"
            "    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"3.0\"/>"
            "  </joint>"
            "  <joint name=\"joint2\" type=\"revolute\">"
            "    <parent link=\"link1\"/><child link=\"link2\"/>"
            "    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"3.0\"/>"
            "  </joint>"
            "</robot>";
            
        const std::string srdf_xml =
            "<?xml version=\"1.0\" ?>"
            "<robot name=\"test_robot\">"
            "  <group name=\"arm\"><joint name=\"joint1\"/><joint name=\"joint2\"/></group>"
            "</robot>";

        urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml);
        auto srdf_model = std::make_shared<srdf::Model>();
        srdf_model->initString(*urdf_model, srdf_xml);
        robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
        jmg = robot_model->getJointModelGroup("arm");
    }

    robot_trajectory::RobotTrajectory createSimpleTrajectory(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities) {

        robot_trajectory::RobotTrajectory traj(robot_model, jmg);
        for (size_t i = 0; i < times.size(); ++i) {
            auto state = std::make_shared<moveit::core::RobotState>(robot_model);
            std::vector<double> pos = {positions[0][i], positions[1][i]};
            std::vector<double> vel = {velocities[0][i], velocities[1][i]};
            state->setJointGroupPositions(jmg, pos);
            state->setJointGroupVelocities(jmg, vel);
            
            double dt = (i == 0) ? 0.0 : (times[i] - times[i-1]);
            traj.addSuffixWayPoint(state, dt);
        }
        return traj;
    }
};

// TC-01: Single-Segment Compliant
TEST_F(SplineSmoothingTest, SingleSegmentCompliant) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0, 2.0},
        {{0.0, 0.5, 1.0}, {0.0, 0.5, 1.0}},
        {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}}
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(traj.getWayPointDurationFromStart(2), 2.0);
    EXPECT_DOUBLE_EQ(res.segment_time_stretch_factors[0], 1.0);
}

// TC-02: Multi-Segment Compliant
TEST_F(SplineSmoothingTest, MultiSegmentCompliant) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0, 2.0, 3.0, 4.0},
        {{0.0, 0.5, 1.0, 1.5, 2.0}, {0.0, 0.5, 1.0, 1.5, 2.0}},
        {{0.0, 1.0, 0.0, 1.0, 0.0}, {0.0, 1.0, 0.0, 1.0, 0.0}}
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(traj.getWayPointDurationFromStart(4), 4.0);
    EXPECT_DOUBLE_EQ(res.segment_time_stretch_factors[0], 1.0);
    EXPECT_DOUBLE_EQ(res.segment_time_stretch_factors[1], 1.0);
}

// TC-03: Single-Segment Acceleration Rescue
TEST_F(SplineSmoothingTest, SingleSegmentAccelRescue) {
    auto traj = createSimpleTrajectory(
        {0.0, 0.2, 0.4},
        {{0.0, 0.5, 1.0}, {0.0, 0.0, 0.0}},
        {{0.0, 2.5, 0.0}, {0.0, 0.0, 0.0}}
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_GT(res.segment_time_stretch_factors[0], 1.0);
    EXPECT_GT(traj.getWayPointDurationFromStart(2), 0.4);
    EXPECT_LE(res.peak_accelerations[0], limits.a_max[0] + 1e-4);
}

// TC-04: Multi-Segment Isolated Rescue
TEST_F(SplineSmoothingTest, MultiSegmentIsolatedRescue) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0, 2.0, 2.15, 2.30},
        {{0.0, 0.5, 1.0, 1.5, 2.0}, {0.0, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.5, 0.0, 3.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}} 
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(res.segment_time_stretch_factors[0], 1.0);
    EXPECT_GT(res.segment_time_stretch_factors[1], 1.0);
    EXPECT_DOUBLE_EQ(traj.getWayPointDurationFromStart(2), 2.0); 
    EXPECT_GT(traj.getWayPointDurationFromStart(4), 2.30);
}

// TC-05: Multi-Segment Independent Rescue
TEST_F(SplineSmoothingTest, MultiSegmentIndependentRescue) {
    auto traj = createSimpleTrajectory(
        {0.0, 0.2, 0.4, 0.6, 0.8}, 
        {{0.0, 0.8, 1.6, 2.4, 3.1}, {0.0, 0.0, 0.0, 0.0, 0.0}},
        {{0.0, 3.5, 0.0, 4.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}} 
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_GT(res.segment_time_stretch_factors[0], 1.0);
    EXPECT_GT(res.segment_time_stretch_factors[1], 1.0);
    EXPECT_NE(res.segment_time_stretch_factors[0], res.segment_time_stretch_factors[1]);
}

// TC-06: Over-Cap Rejection
TEST_F(SplineSmoothingTest, OverCapRejection) {
    auto traj = createSimpleTrajectory(
        {0.0, 0.01, 0.02},
        {{0.0, 1.0, 2.0}, {0.0, 0.0, 0.0}},
        {{0.0, 10.0, 0.0}, {0.0, 0.0, 0.0}}
    );
    config.max_time_stretch = 1.20;
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::EXCEEDS_MAX_TIME_STRETCH);
}

// TC-07: Position Band Violation
TEST_F(SplineSmoothingTest, PositionBandViolation) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0, 2.0},
        {{3.13, 3.13, 3.13}, {0.0, 0.0, 0.0}},
        {{0.0, 2.0, 0.0}, {0.0, 0.0, 0.0}} 
    );

    limits.p_max = {3.10, 3.10};

    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::POSITION_LIMIT_VIOLATION);
}

// TC-08: Path Deviation Violation
TEST_F(SplineSmoothingTest, PathDeviationViolation) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0, 2.0},
        {{0.0, 1.0, 2.0}, {0.0, 0.0, 0.0}},
        {{0.0, -5.0, 0.0}, {0.0, 0.0, 0.0}}
    );

    config.path_dev_max = 0.001;

    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::PATH_DEVIATION_VIOLATION);
}

// TC-09: Truncated Grid (PILZ 44ms Scenario)
TEST_F(SplineSmoothingTest, TruncatedGridPilz) {
    auto traj = createSimpleTrajectory(
        {0.0, 0.1, 0.2, 0.244},
        {{0.0, 0.1, 0.2, 0.22}, {0.0, 0.0, 0.0, 0.0}},
        {{0.0, 1.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::SUCCESS);
    EXPECT_GT(res.segment_time_stretch_factors[0], 1.0);
}

// TC-10: Seam Detection Sensitivity
TEST_F(SplineSmoothingTest, SeamDetectionSensitivity) {
    std::vector<std::vector<double>> vels = {
        {0.0, 1e-7, 1e-5, 0.0} 
    };
    std::vector<size_t> seams = findSeams(vels, 1e-6);
    
    EXPECT_EQ(seams.size(), 3);
    EXPECT_EQ(seams[0], 0);
    EXPECT_EQ(seams[1], 1);
    EXPECT_EQ(seams[2], 3); 
}

// TC-11: Degenerate Input
TEST_F(SplineSmoothingTest, DegenerateInput) {
    auto traj = createSimpleTrajectory(
        {0.0, 1.0},
        {{0.0, 1.0}, {0.0, 1.0}},
        {{0.0, 0.0}, {0.0, 0.0}}
    );
    SmoothingResult res = processTrajectory(config, limits, traj);
    
    EXPECT_EQ(res.status, SmoothingStatus::INVALID_INPUT);
}