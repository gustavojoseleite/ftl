// Contrato das mensagens do ftl_msgs, visto de C++ (R3, R4, R5).
//
// Roda no container, pelo ~/dev/ros2/scripts/test.sh, contra o código gerado pelo rosidl.

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

#include "ftl_msgs/msg/follower_status.hpp"
#include "ftl_msgs/msg/progress.hpp"
#include "ftl_msgs/msg/rail.hpp"
#include "ftl_msgs/msg/rail_point.hpp"
#include "ftl_msgs/msg/tracked_vehicle.hpp"
#include "ftl_msgs/msg/v2_v_envelope.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using ftl_msgs::msg::FollowerStatus;
using ftl_msgs::msg::Progress;
using ftl_msgs::msg::Rail;
using ftl_msgs::msg::RailPoint;
using ftl_msgs::msg::TrackedVehicle;
using ftl_msgs::msg::V2VEnvelope;

namespace
{

// Serializa e lê de volta, como a mensagem faria ao passar pelo DDS.
template<typename T>
T ida_e_volta(const T & original)
{
  rclcpp::Serialization<T> serializador;
  rclcpp::SerializedMessage bruto;
  serializador.serialize_message(&original, &bruto);
  T lida;
  serializador.deserialize_message(&bruto, &lida);
  return lida;
}

}  // namespace

TEST(Constantes, FontesDePosicao)
{
  EXPECT_EQ(TrackedVehicle::SOURCE_NONE, 0);
  EXPECT_EQ(TrackedVehicle::SOURCE_GNSS, 1);
  EXPECT_EQ(TrackedVehicle::SOURCE_TAG, 2);
}

TEST(Constantes, EstadosDoSupervisor)
{
  EXPECT_EQ(FollowerStatus::STATE_INITIALIZING, 0);
  EXPECT_EQ(FollowerStatus::STATE_WAITING_FOR_RAIL, 1);
  EXPECT_EQ(FollowerStatus::STATE_NOMINAL, 2);
  EXPECT_EQ(FollowerStatus::STATE_TAG_ONLY, 3);
  EXPECT_EQ(FollowerStatus::STATE_GNSS_ONLY, 4);
  EXPECT_EQ(FollowerStatus::STATE_ESTIMATED_POSE, 5);
  EXPECT_EQ(FollowerStatus::STATE_LOCAL_TRAJECTORY, 6);
  EXPECT_EQ(FollowerStatus::STATE_SAFE_STOP, 7);
  EXPECT_EQ(FollowerStatus::STATE_EMERGENCY, 8);
}

TEST(Constantes, TiposDoEnvelope)
{
  EXPECT_EQ(V2VEnvelope::KIND_FIX, 1);
  EXPECT_EQ(V2VEnvelope::KIND_STATUS, 2);
  EXPECT_EQ(V2VEnvelope::KIND_RAIL, 3);
}

TEST(Envelope, LevaUmConteudoSo)
{
  V2VEnvelope e;
  e.fix.push_back(sensor_msgs::msg::NavSatFix());
  EXPECT_THROW(e.fix.push_back(sensor_msgs::msg::NavSatFix()), std::length_error);
}

TEST(Serializacao, TrilhoIdaEVoltaPeloEnvelope)
{
  Rail trilho;
  trilho.leader_id = "pickup";
  trilho.built_by = "tractor_1";
  for (uint32_t i = 0; i < 3; ++i) {
    RailPoint p;
    p.index = i;
    p.s = 0.5 * i;
    p.curvature = 0.04;
    p.speed_limit = 3.0;
    trilho.points.push_back(p);
  }
  V2VEnvelope e;
  e.sender = "tractor_1";
  e.sequence = 42u;
  e.kind = V2VEnvelope::KIND_RAIL;
  e.rail.push_back(trilho);

  const V2VEnvelope volta = ida_e_volta(e);
  EXPECT_EQ(volta.sender, "tractor_1");
  EXPECT_EQ(volta.sequence, 42u);
  EXPECT_EQ(volta.kind, V2VEnvelope::KIND_RAIL);
  EXPECT_TRUE(volta.fix.empty());
  EXPECT_TRUE(volta.status.empty());
  ASSERT_EQ(volta.rail.size(), 1u);
  ASSERT_EQ(volta.rail[0].points.size(), 3u);
  EXPECT_DOUBLE_EQ(volta.rail[0].points[2].s, 1.0);
  EXPECT_EQ(volta.rail[0].built_by, "tractor_1");
}

TEST(Serializacao, EstadoEProgressoIdaEVolta)
{
  FollowerStatus st;
  st.vehicle_id = "tractor_2";
  st.state = FollowerStatus::STATE_SAFE_STOP;
  st.reason = "perdeu o GNSS e a tag";
  st.gap_error = -1.5;
  const FollowerStatus st_volta = ida_e_volta(st);
  EXPECT_EQ(st_volta.state, FollowerStatus::STATE_SAFE_STOP);
  EXPECT_EQ(st_volta.reason, "perdeu o GNSS e a tag");
  EXPECT_DOUBLE_EQ(st_volta.gap_error, -1.5);

  Progress pr;
  pr.vehicle_id = "tractor_3";
  pr.s = 37.25;
  pr.released_index = 120u;
  const Progress pr_volta = ida_e_volta(pr);
  EXPECT_DOUBLE_EQ(pr_volta.s, 37.25);
  EXPECT_EQ(pr_volta.released_index, 120u);
}
